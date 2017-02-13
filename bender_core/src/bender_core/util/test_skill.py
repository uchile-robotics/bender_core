#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill


class TestSkill(RobotSkill):

    _type = "test_skill"

    """
    The TestSkill is intented to run tests on the robot_skill module.

    You will need to replace the _type and dependency register on runtime
    to fill your needs

    e.g:
    >>> import rospy
    >>> from bender_skills.robot import Robot
    >>> from bender_skills.util.test_skill import TestSkill
    >>> A = TestSkill("A")
    >>> B = TestSkill("B")
    >>> C = TestSkill("C")
    >>> D = TestSkill("D")
    >>> E = TestSkill("E")
    >>> F = TestSkill("F")
    >>> G = TestSkill("G")

    >>> A.register_dependency(C.get_type())
    >>> A.register_dependency(D.get_type())
    >>> A.register_dependency(E.get_type())
    >>> B.register_dependency(C.get_type())
    >>> C.register_dependency(F.get_type())
    >>> D.register_dependency(E.get_type())
    >>> D.register_dependency(G.get_type())

    >>> robot = Robot()
    >>> robot.set(A)
    >>> robot.set(B)
    >>> robot.set(C)
    >>> robot.set(D)
    >>> robot.set(E)
    >>> robot.set(F)
    >>> robot.set(G)
    """

    def __init__(self, custom_type):
        super(TestSkill, self).__init__()
        self._type = custom_type
        self._description = "a test skill named (%s)" % self._type

        self.pause_return_value = True
        self.start_return_value = True
        self.check_return_value = True
        self.setup_return_value = True
        self.shutdown_return_value = True

    # override parent method!
    def get_type(self):
        return self._type


    def check(self):
        rospy.loginfo("{skill: %s}: check()." % self._type)
        return self.check_return_value

    
    def setup(self):
        rospy.loginfo("{skill: %s}: setup()." % self._type)
        return self.setup_return_value


    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return self.shutdown_return_value
        

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return self.start_return_value


    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)
        return self.pause_return_value

