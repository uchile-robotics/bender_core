#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill
from std_msgs.msg import Bool, UInt8
from bender_msgs.msg import FaceEmotion

class FaceSkill(RobotSkill):
    """
    Face interface for emotions and mouth.
    """
    _type = "face"

    def __init__(self):
        super(FaceSkill, self).__init__()
        self._description = "face skill"

        self._topic_brighness = '/bender/led_head_controller/brightness'
        self._topic_emotion = '/bender/led_head_controller/emotion_cmd'
        self._topic_mouth = '/bender/led_head_controller/move_mouth'

        self._brighness_pub = None
        self._emotion_pub = None
        self._mouth_pub = None

        self.emotion_msg = FaceEmotion()
        self.brightness_msg = UInt8()
        self.mouth_msg = Bool()
        self.mouth_msg.data = True

    def check(self, timeout = 1.0):
        # TODO
        return True
    
    def setup(self):
        self._brighness_pub = rospy.Publisher(self._topic_brighness, UInt8, queue_size=5)
        self._emotion_pub = rospy.Publisher(self._topic_emotion, FaceEmotion, queue_size=5)
        self._mouth_pub = rospy.Publisher(self._topic_mouth, Bool, queue_size=5)
        return True

    def shutdown(self):
        self.loginfo('Calling \'light_off\'.')
        self.emotion_msg.emotion_name = 'light_off'
        self._emotion_pub.publish(self.emotion_msg)
        return True

    def start(self):
        return True

    def pause(self):
        return True

    def turn_off(self):
        self.set_emotion('light_off')

    def set_emotion(self, name='lantern1'):
        """
        Send emotion to face.

        Args:
            name (String): Emotion name.

        Examples:
            >>> robot.face.set_emotion('happy1')
        """
        self.emotion_msg.emotion_name = name
        self._emotion_pub.publish(self.emotion_msg)


    def set_brightness(self, intensity=15):
        """
        Set the led brightness.

        Args:
            intensity (int): Led brightness between [0-255]

        Examples:
            >>> robot.face.set_brightness(150)
        """
        sat_value = min(255, max(0, intensity))
        self.brightness_msg.data = sat_value
        self._brighness_pub.publish(self.brightness_msg)

    def move_mouth(self):
        """
        Move the robot mouth once.

        Examples:
            >>> robot.face.move_mouth()
        """
        self._mouth_pub.publish(self.mouth_msg)
