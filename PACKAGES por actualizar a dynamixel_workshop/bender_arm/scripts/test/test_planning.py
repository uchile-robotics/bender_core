#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import Pose, PoseStamped, Point
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


def main():
    # wrt base_link
    wrt_bl = PoseStamped()
    wrt_bl.header.frame_id = 'bender/base_link'
    wrt_bl.pose = getPose(x=0.3, y=-0.3, z=0.8)

    # wrt neck_link
    wrt_nl = PoseStamped()
    wrt_nl.header.frame_id = 'bender/neck_link'
    wrt_nl.pose = getPose(x=0.35, y=-0.3, z=-0.55)

    r_limb = Limb('r')
    
    #r_limb.gripper.close()
    
    #r_limb.arm.set_planner('LBKPIECEkConfigDefault') # Ver otras configs en bender_moveit_config/config/ompl_planning.yaml

    # r_limb.arm.set_position_named('home')
    # rospy.sleep(1.0)

    r_limb.arm.set_position(wrt_nl)
    #rospy.sleep(2.0)
    r_limb.arm.set_position_named('home')
    #rospy.sleep(2.0)
    r_limb.arm.set_position(wrt_bl)
    wrt_bl.pose = getPose(x=0.3, y=-0.3, z=0.8,pitch=-0.1)
    r_limb.arm.set_position(wrt_bl)
    wrt_bl.pose = getPose(x=0.3, y=-0.3, z=0.8,pitch=-0.2)
    r_limb.arm.set_position(wrt_bl)
    wrt_bl.pose = getPose(x=0.35, y=-0.35, z=0.85,pitch=-0.1)
    r_limb.arm.set_position(wrt_bl)
    wrt_bl.pose = getPose(x=0.35, y=-0.35, z=0.85,pitch=-0.1,yaw=0.1)
    r_limb.arm.set_position(wrt_bl)
    wrt_bl.pose = getPose(x=0.35, y=-0.35, z=0.85,pitch=-0.1,yaw=-0.1)
    r_limb.arm.set_position(wrt_bl)
    
    #r_limb.arm.orientate_gripper()
    #r_limb.gripper.open()
    #rospy.sleep(1.0)
    #r_limb.gripper.close()
    # rospy.sleep(2.0)
    # r_limb.arm.set_position_named('home')
    # rospy.sleep(2.0)

    # Approx IK
    #wrt_bl.pose.position.z += 0.05
    #wrt_bl.pose.position.x += 0.02

    # r_limb.arm.set_position(wrt_bl)
    # rospy.sleep(2.0)
    
    # r_limb.arm.orientate_gripper()
    # rospy.sleep(2.0)
    # r_limb.arm.set_position_named('home')

    # r_limb.arm.servoing(x=0.5,z=0.1)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(y=0.03)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(z=0.05)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(z=-0.05)

    # r_limb.arm.servoing(x=-0.05)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(y=-0.03)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(z=0.05)
    # rospy.sleep(1.0)
    # r_limb.arm.servoing(z=-0.05)

    # r_limb.arm.go_home()

if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()