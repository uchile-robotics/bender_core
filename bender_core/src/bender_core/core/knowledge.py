#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill



class KnowledgeSkill(RobotSkill):
    """
    The KnowledgeSkill

    TODO
    """
    _type = "knowledge"

    def __init__(self):
        super(KnowledgeSkill, self).__init__()
        self._description = "the knowledge skill" 
        
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

