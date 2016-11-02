#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill



class SoundSkill(RobotSkill):
    """
    The SoundSkill

    TODO
    """
    _type = "sound"

    def __init__(self):
        super(SoundSkill, self).__init__()
        self._description = "the sound skill"
        
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

