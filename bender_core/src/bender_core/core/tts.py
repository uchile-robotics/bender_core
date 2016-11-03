#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill
from bender_srvs.srv import String
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class TTSSkill(RobotSkill):
    """
    The TTSSkill

    TODO
    """
    _type = "tts"

    def __init__(self):
        super(TTSSkill, self).__init__()
        self._description = "the tts skill"

        self._tts_topic = "/bender/hw/tts/say"
        self._set_language_topic = "/bender/hw/tts/set_language"
        self._stop_tts_topic = "/bender/hw/tts/stop_speech"
        self._is_talking_topic = "/bender/hw/head/move_mouth"

        self._tts_client = None
        self._set_language_client = None
        self._stop_tts_client = None

        self.is_talking = False


        
    def check(self, timeout = 1.0):
        # rospy.loginfo("{skill: %s}: check()." % self._type)

        try:
            rospy.wait_for_service(self._tts_topic, timeout)
        except rospy.ROSException:
            self.logerr("synthesize server not found")
            return False

        try:
            rospy.wait_for_service(self._set_language_topic, timeout)
        except rospy.ROSException:
            self.logerr("set_language server not found")
            return False

        try:
            rospy.wait_for_service(self._stop_tts_topic, timeout)
        except rospy.ROSException:
            self.logerr("shut_up server not found")
            return False

        self.logdebug("set_language server [OK]")
        self.logdebug("synthesizer server [OK]")
        self.logdebug("shut_up server [OK]")
        return True
    
    def setup(self):
        # rospy.loginfo("{skill: %s}: setup()." % self._type)
        self._tts_client = rospy.ServiceProxy(self._tts_topic, String)
        self._set_language_client = rospy.ServiceProxy(self._set_language_topic, String)
        self._stop_tts_client = rospy.ServiceProxy(self._stop_tts_topic, Empty)
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

    def say(self, text="bender yes"):
        """
        Send text to the tts system to speak it out loud

        Args:
            text (String): Text to synthesize.  Default: "bender yes"

        Examples:
            >>> robot.tts.say("Hello, my name is bender")
        """
        try:
            self._tts_client(text)
        except rospy.ROSException:
            self.logerr("could not call synthesizer server")
            return False

        return True


    def set_language(self, language="english"):
        """
        Set the language for tts system. Available languages: english/spanish

        Args:
            language (string): Bender's language. Default: "english"

        Examples:
            >>> robot.tts.set_language("spanish")
        """
        try:
            self._set_language_client(language)
        except rospy.ROSException:
            self.logerr("could not call set_language server")
            return False

        return True

    def stop(self):
        """
        Stop speech audio stream

        Args:
            None

        Examples:
            >>> robot.tts.stop()
        """
        try:
            self._stop_tts_client()
        except rospy.ROSException:
            self.logerr("could not call stop_speech server")
            return False

        return True

    def wait_until_done(self,timeout=10.0):
        begin = rospy.get_time()
        
        self.is_talking = True
        rospy.sleep(1)
        rospy.Subscriber(self._is_talking_topic, Bool, self.is_talking_callback)
        while(not rospy.is_shutdown() and self.is_talking  and (rospy.get_time()-begin) < timeout):
            pass
        return


    def is_talking_callback(self,data):
        
        self.is_talking = data.data
        