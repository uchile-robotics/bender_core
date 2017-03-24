#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from bender_joy import xbox

class JoystickBase(object):
  
    def __init__(self):
        rospy.loginfo('Joystick base init ...')

        self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)
        self.cancel_goal_client = rospy.ServiceProxy('/bender/nav/goal_server/cancel', Empty)

        # control
        self.is_paused = False

        # load configuration
        self.b_pause    = rospy.get_param('~b_pause', 'START')
        self.b_cancel   = rospy.get_param('~b_cancel', 'B')
        a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
        a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')
        self.max_linear_vel  = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)
        
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause   = key_mapper.get_button_id(self.b_pause)
        self.b_idx_cancel  = key_mapper.get_button_id(self.b_cancel)
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


    # this method breaks the decoupling between base and soft ws!!!
    def cancel_goal(self):
        try:
            self.cancel_goal_client.wait_for_service(0.5)
            self.cancel_goal_client()
            rospy.loginfo("Goal cancelled")
        except rospy.ServiceException, e:
            rospy.loginfo("There is no goal to cancel")
            pass


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

        elif msg.buttons[self.b_idx_cancel]:
            self.cancel_goal()
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
