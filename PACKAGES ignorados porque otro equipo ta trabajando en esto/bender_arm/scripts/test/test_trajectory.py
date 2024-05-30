#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Arm, Gripper

def main():
    arm = Arm('l_arm')
    #gripper = Gripper('l_gripper')
    Gripper('l_gripper')
    arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0], interval = 0.3)
    arm.wait()
    arm.move_joint([0.0, 0.14, 0.16, 1.40, -0.06, 0.22], interval = 3, segments = 50)
    arm.wait()
    arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0], interval = 4, segments = 30)
    # gripper.command(position=0.0, effort=700)
    # arm.move_trajectory([[-0.051,0.129,-0.061,0.021,-0.026,-0.061],
    #   [-0.293,0.062,-0.435,0.548,0.378,0.251],
    #   [-0.495,0.117,-0.271,1.269,0.026,0.031],
    #   [-0.31,0.352,0.041,1.313,-0.031,0.118],
    #   [0.083,0.571,0.685,1.348,-0.036,0.199],
    #   [0.483,0.584,1.104,1.285,0.015,0.399],
    #   [0.606,0.275,0.598,1.186,-0.102,0.291],
    #   [0.615,0.189,0.261,1.192,-0.256,0.133],
    #   [0.529,0.196,0.24,1.143,-0.307,-0.225],
    #   [0.183,0.273,0.271,1.141,-0.261,0.097],
    #   [-0.051,0.319,0.24,1.088,-0.194,0.235],
    #   [-0.19,0.3,0.245,0.891,-0.189,0.271],
    #   [-0.233,0.226,0.205,0.485,-0.189,0.337],
    #   [-0.087,0.158,0.179,0.092,-0.194,0.061],
    #   [-0.055,0.026,0.143,0.069,-0.194,-0.082]])
    
    # rospy.sleep(3.0)
    # gripper.command(position=0.5, effort=700)
    # gripper.wait()
    # gripper.close()
    # arm.wait()
    #gripper = Gripper('l_gripper')
    #gripper.command(position=0.0, effort=700)
    # arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
    # print arm.wait()
    # rospy.sleep(2.0)
    # arm.move_joint([0.0, 0.14, 0.16, 1.40, -0.06, 0.22], interval = 1.5)
    # raw_input('Press [ENTER] to go home...')
    # arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0], interval = 1.5)
    # print arm.wait()
    # rospy.sleep(2.0)
    # #gripper.wait()
    #arm.move_joint([0.006135923151542565, 0.04918249202118943, -0.06135923151542565, 1.3125050568552423, -0.02556634646476069, -0.05113269292952138])
    #gripper.command(position=0.8, effort=700)
    #gripper.wait()
    #rospy.sleep(2.0)
    #gripper.command(position=0.0, effort=700)
    #gripper.wait()
    #rospy.sleep(2.0)
    #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
  
if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
