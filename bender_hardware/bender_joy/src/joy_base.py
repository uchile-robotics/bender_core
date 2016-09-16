#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from bender_joy import xbox

class JoystickBase(object):
  
    def __init__(self):
        rospy.loginfo('Joystick base init ...')

        self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)

        # control
        self.is_paused = False

        # load configuration
        self.b_pause    = rospy.get_param('~b_pause', 'START')
        a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
        a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')
        max_linear_vel  = rospy.get_param('~max_linear_vel', 0.5)
        max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)
        
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause   = key_mapper.get_button_id(self.b_pause)
        self.a_idx_linear  = key_mapper.get_axis_id(a_linear)
        self.a_idx_angular = key_mapper.get_axis_id(a_angular)


        # check
        self.assert_params()


        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for base is ready')


    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_angular, int)
        assert isinstance(self.a_idx_linear, int)


    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:                

                # stop signal
                cmd = Twist()
                self.pub.publish(cmd)

                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:
            cmd = Twist()
            cmd.angular.z = self.max_angular_vel*msg.axes[self.a_idx_angular]
            cmd.linear.x = self.max_linear_vel*msg.axes[self.a_idx_linear]
            self.pub.publish(cmd)
        


if __name__ == '__main__':
    rospy.init_node('joy_base')
    JoystickBase()
    rospy.spin()
