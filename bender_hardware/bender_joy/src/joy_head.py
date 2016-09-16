#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
import math
from sensor_msgs.msg import Joy
from bender_msgs.msg import Emotion
from bender_joy import xbox

class JoystickHead(object):
  
    def __init__(self):
        rospy.loginfo('Joystick head init ...')

        # connections
        self.head_pub   = rospy.Publisher('head/cmd', Emotion, queue_size=1)


        # load head configuration
        self.b_pause = rospy.get_param('~b_pause', 'START')
        
        b_neck_actuate = rospy.get_param('~b_neck_actuate', 'RS')
        a_neck_sides   = rospy.get_param('~a_neck_sides', 'RS_HORZ')
        a_neck_front   = rospy.get_param('~a_neck_front', 'RS_VERT')
        
        b_increment    = rospy.get_param('~b_increment', 'RB')
        b_decrement    = rospy.get_param('~b_decrement', 'LB')
        b_happy        = rospy.get_param('~b_happy', 'A')
        b_angry        = rospy.get_param('~b_angry', 'B')
        b_sad          = rospy.get_param('~b_sad', 'X')
        b_surprise     = rospy.get_param('~b_surprise', 'Y')

        self.max_neck_degrees      = rospy.get_param('~max_neck_degrees', 60)
        self.max_emotion_intensity = rospy.get_param('~max_emotion_intensity', 3)
        self.min_emotion_intensity = rospy.get_param('~min_emotion_intensity', 1)

        # convert to ids
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.a_idx_neck_sides   = key_mapper.get_axis_id(a_neck_sides)
        self.a_idx_neck_front   = key_mapper.get_axis_id(a_neck_front)
        self.b_idx_neck_actuate = key_mapper.get_button_id(b_neck_actuate)
        self.b_idx_increment    = key_mapper.get_button_id(b_increment)
        self.b_idx_decrement    = key_mapper.get_button_id(b_decrement)
        self.b_idx_happy        = key_mapper.get_button_id(b_happy)
        self.b_idx_angry        = key_mapper.get_button_id(b_angry)
        self.b_idx_sad          = key_mapper.get_button_id(b_sad)
        self.b_idx_surprise     = key_mapper.get_button_id(b_surprise)

        # check
        self.assert_params()
        

        # control
        self.is_paused = False
        self.emotion_intensity = self.min_emotion_intensity


        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for head is ready')


    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_neck_sides, int)
        assert isinstance(self.a_idx_neck_front, int)
        assert isinstance(self.b_idx_neck_actuate, int)
        assert isinstance(self.b_idx_increment, int)
        assert isinstance(self.b_idx_decrement, int)
        assert isinstance(self.b_idx_happy, int)
        assert isinstance(self.b_idx_angry, int)
        assert isinstance(self.b_idx_sad, int)
        assert isinstance(self.b_idx_surprise, int)


    def move_head(self, degrees):

        # saturate
        degrees = max(-self.max_neck_degrees, degrees)
        degrees = min(self.max_neck_degrees, degrees) 

        # send
        msg = Emotion()
        msg.Order = "MoveX"
        msg.Action = ""
        msg.X = degrees
        self.head_pub.publish(msg)

        # todo: throttle 0.5s
        rospy.loginfo("Setting neck angle to %f [deg]" % degrees)


    def set_emotion(self, emotion_str):
        msg = Emotion()
        msg.Order = "changeFace"
        msg.Action = emotion_str
        msg.X = 0
        self.head_pub.publish(msg)
        rospy.loginfo("Setting head emotion to %s" % emotion_str)


    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:                
                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:

            # neck
            if msg.buttons[self.b_idx_neck_actuate]:
                neck_side  = msg.axes[self.a_idx_neck_sides]
                neck_front = msg.axes[self.a_idx_neck_front]

                if neck_side*neck_side + neck_front*neck_front > 0.5:
                    angle = math.atan2(neck_side, neck_front)
                    degrees = math.degrees(angle)
                    self.move_head(degrees)

                return

            # head emotion in/decrement
            elif msg.buttons[self.b_idx_decrement]:
                self.emotion_intensity = int(max(self.min_emotion_intensity, self.emotion_intensity - 1))
                rospy.loginfo("Emotion intensity decrement: %d" % self.emotion_intensity)
            elif msg.buttons[self.b_idx_increment]:
                self.emotion_intensity = int(min(self.max_emotion_intensity, self.emotion_intensity + 1))
                rospy.loginfo("Emotion intensity increment: %d" % self.emotion_intensity)
            

            # head emotions: happy, angry, sad, surprise
            elif msg.buttons[self.b_idx_happy]:
                self.set_emotion("happy%d" % self.emotion_intensity)
            
            elif msg.buttons[self.b_idx_angry]:
                self.set_emotion("angry%d" % self.emotion_intensity)
            
            elif msg.buttons[self.b_idx_sad]:
                self.set_emotion("sad%d" % self.emotion_intensity)
            
            elif msg.buttons[self.b_idx_surprise]:
                self.set_emotion("surprise")

            return


if __name__ == '__main__':
    rospy.init_node('joy_head')
    JoystickHead()
    rospy.spin()
