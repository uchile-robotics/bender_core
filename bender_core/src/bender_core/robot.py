#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
__author__ = 'Matías Pavez'
__email__  = 'matias.pavez@ing.uchile.cl'

import rospy
from .robot_skill import RobotSkill
# Context
from .context import Context


class Robot(object):
    """
    Robot is a container/manager of all the skills a robot can be built of.
    The robot maintains a dictionary of skills, whose keys are retrieved from
    the '<robot_skill>.get_type()' method

    Robot class provides the following public API:
    // single skill methods
    - has:   checks for a RobotSkill
    - get:   gets a RobotSkill
    - set:   sets a RobotSkill
    
    // multiple skills
    - check: checks skills integrity
    - setup: setups registered skills
    - start: starts all or a single skill and its dependencies recursively
    - pause: pauses all or a single skill and its parents recursively
    - shutdown: turns off all or a single skill and its parents recursively
    
    Whenever shutdown(), start() or pause() methods are called:
    - they work only on the registered skill graph. Is your responsibility to
     register the required dependencies when building the robot.
    - 
    - except the above point, do not assume there is an order in the calling
     of methods.
    TODO: The same for check() and setup()

    WARNING: Setting skills after any call to check(), setup(), start(),
     pause()  or shutdown() is not currently supported!. Bad stuff will 
     happen!.


    example:
    # build the robot
    >>> bender = Robot("manipulator bot")
    >>> bender = set(ArmSkill.get_instance())
    >>> bender = set(GripperSkill.get_instance())
    >>> bender = set(HeadSkill.get_instance())
    >>> bender = set(RGBDSkill.get_instance())
    >>> bender = set(JoySkill.get_instance())
    >>> print bender

    # prepare
    >>> bender.check()
    >>> bender.setup()

    # start the gripper and its dependencies (arm and joy)
    >>> bender.start(GripperSkill.get_type())

    # do something
    >>> gripper = bender.get(GripperSkill.get_type())
    >>> gripper.some_fantastic_function()
    """


    ## ========================================================================
    ## PUBLIC ROBOT INTERFACE:

    def __init__(self, name):
        self.name = name
        self._skills = dict()
        self.context = Context(robot_name=name)

    def __del__(self):
        pass
        # TODO(mpavez) Good idea? call all shutdown methods when robot object get deleted
        #rospy.logerr("robot shutdown")
        #self.shutdown()

    def has(self, skill_name):
        """
        returns True if there is a RobotSkill named skill_name
        False otherwise
        """
        return skill_name in self._skills


    def get(self, skill_name):
        """
        returns the RobotSkill named skill_name

        if the skill is not found, then a Mock is installed and returned
        """
        if not skill_name in self._skills:
            # self._skills[skill_name] = RobotSkill.createMock(skill_name, skill_name)
            rospy.logerr("robot.get(): robot does not know the '%s' skill." % skill_name)
            return None

        return self._skills[skill_name]


    def __getattr__(self, attr):
        """
        python black magik!
        this hook lets you use the robot as follows:
        - robot.base.move_forward() ...
        - robot.arm. ...
        """
        if self.has(attr):
            return self.get(attr)
        raise AttributeError("'{0}' object has no attribute '{1}'".format(self.__class__.__name__, attr))


    def set(self, skill):
        """
        adds a RobotSkill to the robot or replaces it if it already exists
        
        raises an AssertionError if the skill does not inherits 
        the RobotSkill class
        """
        if not isinstance(skill, RobotSkill):
            raise AssertionError

        rospy.loginfo("robot.set(): setting skill type '%s'." % skill.get_type())
        if skill.get_type() in self._skills:
            rospy.logwarn("robot.set(): '%s' already exists ...replacing." % skill.get_type())

        if skill.robot is None:
            skill._set_robot(self)
        self._skills[skill.get_type()] = skill
        setattr(self, skill.get_type(), skill)


    def check(self):
        """
        checks all robot skills

        returns True on success of every skill, False otherwise
        """
        status = self._prepare_status_()

        # perform checks
        rospy.loginfo("robot check: checking skills...")

        # Usually timeout methods use ros.get_rostime that use subscriber to /clock
        # To avoid errors sleep a little time
        rospy.sleep(0.2)
        for skill in self._skills.itervalues():
            name = skill.get_type()
            # rospy.loginfo("... checking skill: %s" % name)
            if not skill.check():
                status[name] = False

        # report results
        self._print_status_(status, "check")
        return self._analize_status_(status)


    def setup(self):
        """
        setups all robot skills

        returns True on success of every skill, False otherwise
        """
        status = self._prepare_status_()

        # perform setups
        rospy.loginfo("robot setup: setting up skills...")
        for skill in self._skills.itervalues():
            name = skill.get_type()
            rospy.loginfo("... setting up skill: %s" % name)
            if not skill.setup():
                status[name] = False

        # report results
        self._print_status_(status, "setup")
        return self._analize_status_(status)


    def start(self, skill_name = None):
        """
        Sends start signals to all registered skills or only one of them
        and its dependencies recursively.
        
        If one skill fails to start, then their parents will not be started!

        returns True on success of every skill, False otherwise

        TODO: add option to force_restart even when already started
        """
        ## start all version
        if skill_name is None:
            succeeded = True
            for skill_name in self._skills.iterkeys():
                succeeded &= self.start(skill_name)
            return succeeded

        ## specific version
        # if already active, then returns
        # else, start childs recursively
        if skill_name not in self._skills:
            rospy.logerr("robot.start(): Unknown skill {%s}. Maybe you forgot to add it to the robot." % skill_name)
            return False
        this = self._skills[skill_name]

        if this.get_state() == RobotSkill._state_offline:
            rospy.logerr("robot.start(): Skill {%s} is already offline, there is no way to turn it on." % skill_name)
            return False

        if this.get_state() == RobotSkill._state_paused:
            
            # start childs recursively
            childs_succeeded = True
            childs = self._skills[skill_name].get_dependencies()
            for child_name in childs:
                childs_succeeded &= self.start(child_name)

            # review childs
            if not childs_succeeded:
                rospy.logerr("robot.start(): For skill {%s} some dependencies failed to start" % skill_name)
                return False

            # start this one
            succeeded = this.start()
            if not succeeded:
                rospy.logerr("robot.start(): Skill {%s} failed to start" % skill_name)
                return False
            this.set_state(RobotSkill._state_active)

        # if this.get_state() == RobotSkill._state_active:
        #     rospy.logwarn("robot.start(): Skill {%s} is already active. Restart is not supported yet." % skill_name)
        return True


    def pause(self, skill_name = None, whole_tree = False):
        """
        Sends pause signals to all registered skills or only one of them and 
        its PARENTS recursively.

        Where "parents" refers to all skills which depends on the current one.

        If one skill fails to pause, it will be marked as "paused" anyway and will
        proceed with its parents.

        returns True on success of every skill, False otherwise

        TODO: add option to force_pause even when already paused
        """
        ## pause all version
        if skill_name is None:
            succeeded = True
            for skill_name in self._skills.iterkeys():
                succeeded &= self.pause(skill_name)
            return succeeded

        ## specific version
        # if already paused, then returns
        # else, pause parents recursively
        if skill_name not in self._skills:
            rospy.logerr("robot.pause(): Unknown skill {%s}. Maybe you forgot to add it to the robot." % skill_name)
            return False
        this = self._skills[skill_name]

        if this.get_state() == RobotSkill._state_offline:
            rospy.logerr("robot.pause(): Skill {%s} is already offline, there is no way to turn it on or pause it." % skill_name)
            return False

        if this.get_state() == RobotSkill._state_active:

            # pause parents recursively
            parents_succeeded = True
            parents = self._get_parents(skill_name)
            for parent_name in parents:
                parents_succeeded &= self.pause(parent_name)

            # review parents
            if not parents_succeeded:
                rospy.logerr("robot.pause(): For skill {%s} some parents failed to pause" % skill_name)

            
            # self pause
            succeeded = this.pause()
            if not succeeded:
                rospy.logwarn("robot.pause(): Skill {%s} failed to pause. Anyway, i will set the skill as paused. Pause signals were sent to its parents" % skill_name)
            this.set_state(RobotSkill._state_paused)

            childs_succeeded = True
            if whole_tree:
                # pause childs recursively
                childs = self._skills[skill_name].get_dependencies()
                for child_name in childs:
                    childs_succeeded &= self.pause(child_name, whole_tree=True)

                # review childs
                if not childs_succeeded:
                    rospy.logerr("robot.pause(): For skill {%s} some dependencies failed to start" % skill_name)

            return succeeded and parents_succeeded and childs_succeeded


        # if this.get_state() == RobotSkill._state_paused:
        #     rospy.logwarn("robot.pause(): Skill {%s} is already paused." % skill_name)
        return True


    def shutdown(self, skill_name = None, whole_tree=False):
        """
        Sends shutdown signals to all registered skills or only one of them and 
        its PARENTS recursively.

        Where "parents" refers to all skills which depends on the current one.

        If one skill fails to be turned off, it will be marked as "offline" 
        anyway and will proceed with its parents.

        returns True on success of every skill, False otherwise

        TODO: add option > force_restart = False
        """
        ## all version
        if skill_name is None:
            succeeded = True
            for skill_name in self._skills.iterkeys():
                succeeded &= self.shutdown(skill_name)
            return succeeded

        ## specific version
        # if already turned off, then returns
        # else, turns off parents recursively
        if skill_name not in self._skills:
            rospy.logerr("robot.shutdown(): Unknown skill {%s}. Maybe you forgot to add it to the robot." % skill_name)
            return False

        this = self._skills[skill_name]
        if not this.get_state() == RobotSkill._state_offline:
            
            # shutdown parents recursively
            parents_succeeded = True
            parents = self._get_parents(skill_name)
            for parent_name in parents:
                parents_succeeded &= self.shutdown(parent_name)

            # review parents
            if not parents_succeeded:
                rospy.logerr("robot.shutdown(): For skill {%s} some parents failed to shutdown" % skill_name)


            # self shutdown
            succeeded = this.shutdown()
            if not succeeded:
                rospy.logwarn("robot.shutdown(): Skill {%s} failed to be turned off. Anyway, i will set the skill as turned off. Shutdown signals were sent to its parents" % skill_name)
            this.set_state(RobotSkill._state_offline)


            childs_succeeded = True
            if whole_tree:
                # pause childs recursively
                childs = self._skills[skill_name].get_dependencies()
                for child_name in childs:
                    childs_succeeded &= self.shutdown(child_name, whole_tree=True)

                # review childs
                if not childs_succeeded:
                    rospy.logerr("robot.shutdown(): For skill {%s} some dependencies failed to start" % skill_name)

            return succeeded and parents_succeeded and childs_succeeded

        return True



    def __repr__(self):
        """
        __repr__ goal is to be an unambiguous version of Robot
        """
        return "%s(%r)" % (self.__class__, self.__dict__)

    def __str__(self):
        """
        __str__ goal is to be a readable version of Robot
        """
        response = "Robot Name: %s\n" % self.name
        response += "Registered skills:\n"
        for skill in self._skills.itervalues():
            #response += " - %s\n" % repr(skill)
            response += " - %s\n" % str(skill)
        return response







    ## ========================================================================
    ## HELPERS: 
    ## DO NOT RELY ON THIS METHODS!, THEY ARE PRIVATE AND COULD CHANGE
    ## AT ANY TIME

    def _get_parents(self, skill_name):
        """
        returns a list with the type of each parent of the provided skill_name
        """
        parents = set()
        for skill in self._skills.itervalues():
            if skill_name in skill.get_dependencies():
                parents.add(skill.get_type())
        return list(parents)

    def _prepare_status_(self):
        return dict(map(lambda x: (x, True), self._skills.keys()))

    def _print_status_(self, status, status_name):
        rospy.loginfo("robot %s reporting results..." % status_name)
        for sample in status.iteritems():
            skill_name = sample[0]
            succeeded  = sample[1]
            if succeeded:
                rospy.loginfo(" - ( OK ) skill (%s) succeeded" % skill_name)
            else:
                rospy.logwarn(" - (FAIL) skill (%s) failed"    % skill_name)
        rospy.loginfo(" ... robot %s finished." % status_name)

    def _analize_status_(self, status):
        return not (False in status.values())







    ## ========================================================================
    ## DEBUGGING:
    ## DO NOT RELY ON THIS METHODS!, THEY ARE PRIVATE AND COULD CHANGE
    ## AT ANY TIME

    def _print_states(self):
        response = ""
        for skill in self._skills.itervalues():
            response += " - %s: %s\n" % (skill.get_type(), skill.get_state_str())
        print response
