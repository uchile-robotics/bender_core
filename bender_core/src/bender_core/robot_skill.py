#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
__author__ = 'Matías Pavez'
__email__  = 'matias.pavez@ing.uchile.cl'

import rospy
import mock
import inspect
from termcolor import colored, cprint
# Custom logging
import logging
# Context
from .context import Context

class Log(object):
    """
    Logger level
    """
    DEBUG = logging.DEBUG
    INFO  = logging.INFO
    WARN  = logging.WARNING
    ERROR = logging.ERROR
    FATAL = logging.CRITICAL

class RobotSkill(object):
    """
    Abstract class for implementing robot skills
    All skills must inherit from this class.

    RobotSkill subclasses (at least) must implement the following methods:
    - __init__() : Setup of skill management variables
    - check(): checks for runtime requirements
    - setup(): sets up the skill runtime requirements
    - start(): send start signals to this module and dependent ones
    - pause() : send pause signals to this module and dependent ones
    - shutdown() : send close signals to this module and dependent ones

    Each skill is identified by its type. All implementations must redefine the
    _type Class variable.
        e.g 
            >>> _type = "manipulation"

    There are 3 states a skill can be on: started, paused and turned off. All 
    skill implementations must start in paused mode. This states are used to
    track the dependency graph and perform start, pause and shutdown task 
    properly.

    The Robot object is the only manager of the dependency graph, so every call
    to start(), pause() or finish() must be done through the Robot class. Avoid
    calling this methods directly on each skill.

        e.g:
            >>> robot = ...
            >>> a_skill = robot.get(a_skill)

            # (discouraged) will pause, but destroys the tracking system!
            >>> a_skill.pause()

            # (correct)
            >>> robot.pause(a_skill)
    """
    
    ## ========================================================================
    ## MUST BE OVERRIDEN

    _type = "unnamed_skill"


    def __init__(self):
        """
        Setups skill management variables.
        
        Implementations must overwrite the following members:
        - _description: The skill description, just for reading purposes

        - _dependencies: Set with skill types this implementation depends on.
            Must be managed using the register_dependency() method.

            About:
            - - - - - -
            Dependencies are a mechanism to propagate signals to all robot 
            modules (see, start, pause(), shutdown()) when required. This way,
            we can free a module and its dependencies recursively, but taking
            care to only operate on each one when they are really freed by its
            parents.

            This mechanism supports the following relationships and constructs
            by nesting them:

            - Module A depends on B
            A ---> B

            - Module A depends on B and C
                /---> B  
            A -|
                \---> C
 
            - Modules A and B depends on C
            A ___
                 \--> C
            B ___/

            - Module A depends on B and C, but B also depends on C
            A -----> B ---- 
               |           |  
               |           v
                ---------> C
            
            This mechanism DOES NOT supports the following relationships!:
            - (cycle) Module A depends on B, B depends on A
            - is necessary to add support for this one?
            A <------> B


            Security:
            - - - - - -
            You can reinforce basic security rules setting some modules as 
            dependencies for skills which can cause problems when out of
            control. For example, its recommended to set the JoySkill as a
            dependency for skills which involve actions (base, navigation,...).


        The following variables are automatically managed by the robot
        container. So DO NOT modify them:!
        - _current_state: current skill state (paused, active, turned_off)

        example:
            >>> self._description = "bender's manipulation capability"
            >>> self.register_dependency("l_arm")
            >>> self.register_dependency("r_arm")
        """
        self._description = "[description should be placed here]"
        self._dependencies = set()
        self._current_state = self._state_paused
        
        # Robot context
        self.robot = None
        self.context =  Context.get_context()
        
        # Config custom logger
        self._log_name = "rosout.{0}.{1}".format(self.context.get_robot_name(),self._type)
        self.log = logging.getLogger(self._log_name)
        self.log.addHandler(rospy.impl.rosout.RosOutHandler())
        # Default logger level at INFO
        self.set_log_level(Log.INFO)
        # Logger alias like rospy
        self.logdebug = self.log.debug
        self.loginfo = self.log.info
        self.logwarn = self.log.warning
        self.logerr = self.log.error
        self.logfatal = self.log.critical


    def set_log_level(self, level):
        """
        Set level to logging functions
        """
        self.log.setLevel(level)


    def check(self):
        """
        Abstract method you must supply when implementing a skill.

        It checks whether runtime requirements are met to use this skill.
        It returns a list with the errors and warnings.

        TODO: list syntax
        """
        raise NotImplementedError("Class %s doesn't implement check()" % (self.__class__.__name__))

    
    def setup(self):
        """
        Abstract method you must supply when implementing a skill.

        Setups the skill runtime requirements
        returns True on success, False otherwise
        """
        raise NotImplementedError("Class %s doesn't implement setup()" % (self.__class__.__name__))


    def start(self):
        """
        Abstract method you must supply when implementing a skill.

        Send start signals to dependent modules.
        For example, actuators, planners and perception routines.
        returns True on success, False otherwise
        """
        raise NotImplementedError("Class %s doesn't implement start()" % (self.__class__.__name__))


    def pause(self):
        """
        Abstract method you must supply when implementing a skill.

        Send pause signals to dependent modules. 
        For example, actuators, planners and perception routines.
        returns True on success, False otherwise
        """
        raise NotImplementedError("Class %s doesn't implement pause()" % (self.__class__.__name__))


    def shutdown(self):
        """
        Abstract method you must supply when implementing a skill.

        Closes the skill runtime requirements
        returns True on success, False otherwise
        """
        raise NotImplementedError("Class %s doesn't implement close()" % (self.__class__.__name__))


    ## ========================================================================
    ## MUST NOT BE OVERRIDEN

    # states of a skill
    _state_paused, _state_active, _state_offline = range(3)

    # debugging!
    _state_dict = {
        _state_paused : "paused",
        _state_active : "active",
        _state_offline: "offline"
    }
    # debugging!
    def get_state_str(self):
        return self._state_dict[self._current_state]

    def __repr__(self):
        """
        __repr__() goal is to be an unambiguous version of the skill
        """
        return "%s(%r)" % (self.__class__, self.__dict__)


    def __str__(self):
        """
        __str__() goal is to be a readable version of the skill
        """
        return "{%s} '%s' ---> %s" % (self._type, self._description, list(self._dependencies))

    def set_state(self, state):
        print "Setting state %s for skill %s" % (self._state_dict[state], self.get_type())
        self._current_state = state

    def get_state(self):
        return self._current_state


    def get_dependencies(self):
        return self._dependencies

    def register_dependency(self, child_type):
        """
        registers a dependency for this skill (one at a time).
        childs cannot be removed.
        """
        self._dependencies.add(child_type)
        return True

    def get_context(self):
        return self.context


    def _set_robot(self, robot):
        if self.robot is None:
            self.robot = robot
        else:
            raise AttributeError("Attempted to set a second robot to this skill!")


    ## ========================================================================
    ## STATIC

    @classmethod
    def overview(cls):
        """
        Prints a description of the skill and available user methods.
        
        Descriptions are taken from the class and classmethod docstrings. Only
        non inherited methods are printed. Overriden methods are excluded.
        """
        _tkey    = lambda x: colored(x, 'cyan')
        _tclass  = lambda x: colored(x, 'blue')
        _tskill  = lambda x: colored(x, 'green')
        _tdepend = lambda x: colored(x, 'yellow')
        _tmethod = lambda x: colored(x, 'blue')
        _tfail   = lambda x: colored(x, 'red')

        cprint(" -- " + cls.__name__ + " INIT -- ", 'green','on_white')
        print _tkey(" - class        : ") + _tclass(cls.__name__)
        print _tkey(" - type         : ") + _tskill(cls.get_type())
        print _tkey(" - dependencies : ") + _tdepend(str(list(cls().get_dependencies())))
        print _tkey(" - class doc    : ")

        # class doc
        class_doc = inspect.getdoc(cls)
        class_doc = '\t' + class_doc.replace('\n','\n\t') + '\n'
        print class_doc
        
        # find user method names
        cls_methods = set(dir(RobotSkill)).symmetric_difference(set(dir(cls)))
        non_private_cls_methods = filter(lambda m: not m.startswith('_'), cls_methods)

        methods_label = _tkey(" - methods      : ")
        if not non_private_cls_methods:
            methods_label += _tfail('[THERE ARE NO AVAILABLE METHODS]')
        else:
            methods_label += str(len(non_private_cls_methods))

        print methods_label
        for method_name in sorted(non_private_cls_methods):
            
            method = getattr(cls, method_name)

            # signature:
            # alternatives:
            #   ugly       - inspect.getargspec(method)
            #   python 3.2 - inspect.signature(method)
            signature = method_name + "(...):"

            # doc
            method_doc = inspect.getdoc(method)

            # handle None case
            if method_doc is None:
                method_doc = ""

            # handle weird cases
            method_doc = method_doc.strip()
            if not method_doc: 
                method_doc = _tfail("[DOCSTRING IS EMPTY!]")

            # indent stuff
            signature  = '    ' + signature
            method_doc = '\t' + method_doc.replace('\n','\n\t')

            print _tmethod(signature)
            print method_doc
            print ""

        cprint(" -- " + cls.__name__ + " END -- ",'green','on_yellow')
            

    @classmethod
    def get_type(cls):
        """
        returns the skill type

        the returned value must be the same, regardless implementation
        e.g: both 'base' and a 'mock base' must return "base"
        """
        return cls._type


    @classmethod
    def get_instance(cls):
        """
        returns an instance of the caller skill class.
        Do not override on subclass!

        e.g:
            >>> from a_head_module import HeadSkill as head
            >>> a_head = head.get_instance()

        TODO: transform in singleton
        """
        return cls()


    @staticmethod
    def create_mock(name, description, depends):
        """
        creates a mock skill for the desired name and description
        only abstract methods are mocked up.

        TODO: convert into classmethod
        """
        # TODO(mpavez) pylint E0202: An attribute affected in bender_skills.robot_skill line 339 hide this method (method-hidden)
        a_skill = RobotSkill()
        a_skill._type = name
        a_skill._description = description
        a_skill.check    = mock.MagicMock(return_value=True)
        a_skill.setup    = mock.MagicMock(return_value=True)
        a_skill.shutdown = mock.MagicMock(return_value=True)
        a_skill.start    = mock.MagicMock(return_value=True)
        a_skill.pause    = mock.MagicMock(return_value=True)
        return a_skill

