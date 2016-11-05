#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Cristopher Gomez, Rodrigo Muñoz"

import rospy

from bender_core.robot_skill import RobotSkill
from bender_srvs.srv import play_sound, play_soundRequest

class SoundSkill(RobotSkill):
    """
    Base class for sound play.
    """
    _type = "sound"

    def __init__(self):
        """
        Base class for sound play.
        """
        super(SoundSkill, self).__init__()
        self._description = "Sound play skill"

        self._sound_topic = "/bender/hw/sound/play"
        self._sound_client = None
        
    def check(self, timeout = 1.0):
        try:
            rospy.wait_for_service(self._sound_topic, timeout)
        except rospy.ROSException:
            self.logerr("Sound service not found")
            return False
        self.logdebug("Sound service [OK]")
        return True
    
    def setup(self):
        self._sound_client = rospy.ServiceProxy(self._sound_topic, play_sound)
        return True

    def shutdown(self):
        self.logwarn("Calling stop sound")
        self.pause()
        return True

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        """
        Pause sound

        Examples:
            >>> robot.sound.pause()
        """
        request = play_soundRequest()
        request.play = False
        try:
            self._sound_client(request)
        except rospy.ServiceException:
            self.logerr("Could not call play sound server")
            return False
        return True

    def play(self, filename="tone"):
        """
        Play sound

        Args:
            filename (String): Filename in folder bender_db/sounds

        Examples:
            >>> robot.sound.play("tone")
        """
        request = play_soundRequest()
        request.play = True
        request.sound_file = filename
        try:
            self._sound_client(request)
        except rospy.ServiceException:
            self.logerr("Could not call play sound server")
            return False
        return True

