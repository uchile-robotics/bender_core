#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Gripper

def main():
    gripper = Gripper('l_gripper')
    gripper.command(position=0.0, effort=0.3)
    print gripper.wait()
    rospy.sleep(1.0)

    print gripper.open()
    rospy.sleep(1.0)

    print gripper.close(effort=0.3)
    rospy.sleep(1.0)

    print gripper.open()
    rospy.sleep(1.0)

    print gripper.close(effort=0.3)
    rospy.sleep(1.0)

    print gripper.open()
    rospy.sleep(1.0)

    print gripper.close(effort=0.3)
    rospy.sleep(1.0)
    
if __name__ == '__main__':
    rospy.init_node('gripper_test')
    main()
