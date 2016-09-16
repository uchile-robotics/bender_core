#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickNavigation(object):
  
    def __init__(self):
        rospy.loginfo('Joystick navigation init ...')

        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        self.pub = rospy.Publisher('navigation/cmd_vel', Twist, queue_size=1)

        # control
        self.is_paused = False

        # load configuration
        self.pause_button_id = rospy.get_param('~pause_button_id', 7)
        self.linear_axe_id   = rospy.get_param('~linear_axe_id', 0)
        self.angular_axe_id  = rospy.get_param('~angular_axe_id', 1)
        self.max_linear_vel  = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)
        
        rospy.loginfo('Joystick navigation is ready')


    def callback(self, msg):

        # pause
        if msg.buttons[self.pause_button_id]:

            self.is_paused = not self.is_paused
            if self.is_paused:                

                # stop signal
                cmd = Twist()
                self.pub.publish(cmd)

                rospy.logwarn("\nControlling PAUSED!, press start button to resume it\n")

            else:
                rospy.logwarn("Controlling RESUMED, press start button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:
            cmd = Twist()
            cmd.angular.z = self.max_angular_vel*msg.axes[self.angular_axe_id]
            cmd.linear.x = self.max_linear_vel*msg.axes[self.linear_axe_id]
            self.pub.publish(cmd)
        


if __name__ == '__main__':
    rospy.init_node('joy_navigation')
    JoystickNavigation()
    rospy.spin()
