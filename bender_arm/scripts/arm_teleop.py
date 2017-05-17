#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm.arm_commander import Limb
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import Joy
from tf import transformations

def getPose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

class SimpleArmTeleop(object):
  
    BUTTONS = {
        'home': 0,
        'middle': 2,
        'change_arm': 6,
        'open_gripper': 4,
        'close_gripper': 5,
        'wave': 1
    }

    """"Control de brazos a posiciones predefinidas"""
    def __init__(self):

        rospy.logwarn('Init arm interface')
        rospy.sleep(10)

        arm_opc = rospy.get_param('/use_arm', 'l_arm')

        self.arms = dict()
        if arm_opc == "both":
            rospy.loginfo('using both')
            self.arms = { 'l':Limb('l'), 'r':Limb('r')}
        
        elif arm_opc == "l_arm":
            rospy.loginfo('using l arm')
            self.arms = { 'l':Limb('l')}
          
        elif arm_opc == "r_arm":
            rospy.loginfo('using r arm')
            self.arms = { 'r':Limb('r')}

        self.selected_arm_side = 'l' 

        # Topico joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.process_joy)
        rospy.loginfo("ArmTeleop OK using arm:" + arm_opc)
        
    def process_joy(self, msg):
        if not self.selected_arm_side in self.arms:
            rospy.logwarn('Selected arm: ' + self.selected_arm_side + ' not available')
            return
        
        selected_arm = self.arms[self.selected_arm_side]
        
        # Home
        if msg.buttons[SimpleArmTeleop.BUTTONS['home']] == 1:
            rospy.logwarn('Arm HOME')
            selected_arm.arm.set_position_named('home')
          
        # Middle
        elif msg.buttons[SimpleArmTeleop.BUTTONS['middle']] == 1:
            rospy.logwarn('Arm MIDDLE')
            selected_arm.arm.set_position_named('middle')
          
        # Cambiar brazo
        elif msg.buttons[SimpleArmTeleop.BUTTONS['change_arm']] == 1:
            if self.selected_arm_side == 'l' and 'r' in self.arms:
                self.selected_arm_side = 'r'
            elif self.selected_arm_side == 'r' and 'l' in self.arms:
                self.selected_arm_side = 'l'
    
            rospy.logwarn('Selected arm: ' + self.selected_arm_side)

        elif msg.buttons[SimpleArmTeleop.BUTTONS['wave']] == 1:
            selected_arm.arm.move_joint([0.0, 0.0, 0.0,1.45,0.0,1.0], interval = 2.5)
            rospy.sleep(1.0)
            selected_arm.arm.move_joint([0.0, 0.0, 0.0,1.45,0.7,1.0], interval = 1.2)
            for i in xrange(5):
                selected_arm.arm.move_joint([0.0, 0.0, 0.0,1.45,0.7,1.0], interval = 0.8)
                rospy.sleep(0.7)
                selected_arm.arm.move_joint([0.0, 0.0, 0.0,1.45,-0.7,1.0], interval = 0.8)
                rospy.sleep(0.7)
            selected_arm.arm.move_joint([0.0, 0.0, 0.0,1.45,0.0,1.0], interval = 2.0)

        # Abrir gripper
        elif msg.buttons[SimpleArmTeleop.BUTTONS['open_gripper']] == 1:
            rospy.logwarn('GRIPPER OPEN')
            selected_arm.gripper.open()
          
        # Close gripper
        elif msg.buttons[SimpleArmTeleop.BUTTONS['close_gripper']] == 1:
            rospy.logwarn('GRIPPER CLOSE')
            selected_arm.gripper.close()

def main():
    SimpleArmTeleop()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('arm_teleop')
    main()
