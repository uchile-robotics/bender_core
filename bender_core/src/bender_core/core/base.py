#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill
from bender_core.core.joy import JoySkill


class BaseSkill(RobotSkill):
    """
    The BaseSkill

    Depends on: Joystick
    """
    _type = "base"

    def __init__(self):
        super(BaseSkill, self).__init__()
        self._description = "the base skill"
        self.register_dependency(JoySkill.get_type())
        
    def check(self):
        rospy.loginfo("{skill: %s}: check()." % self._type)
        return True

    
    def setup(self):
        rospy.loginfo("{skill: %s}: setup()." % self._type)
        return True


    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return True
        

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True


    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)
        return True


    def move_forward(self, distance=0.0):
        """
        This method moves the base forward by "distance" meters.

        Extensive documentation of the move_forward method. This docstring
        will not be displayed when showing an overview of the robot methods.

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        rospy.loginfo("{skill: %s}: move_forward(). Moving forward by %f meters." % (self._type, distance))
        return True

    def move_backwards(self, distance=0.0):
        """
        This method moves the base backwards by "distance" meters.

        Extensive documentation of the move_backwards method. This docstring
        will not be displayed when showing an overview of the robot methods.

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        rospy.loginfo("{skill: %s}: move_backwards(). Moving backwards by %f meters." % (self._type, distance))
        return True

    def _move_forward_private_version(self, distance=0.0):
        """
        Methods starting with '_' will not be displayed in the skill overview
        """
        return True
