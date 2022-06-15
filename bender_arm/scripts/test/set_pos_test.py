#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point





def main():
    
    
    g = MoveGroupInterface("l_arm", "bender/base_link",None,False)  #"bender/l_shoulder_pitch_link"

    joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      'l_wrist_pitch_joint']
    
    rospy.loginfo('INICIA TEST')
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    #----------------------------------------------------------------

    spos=PoseStamped()
    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.3, 0.28, 0.70

    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = -0.0148,-0.733 , -0.0404, 0.678

    spos.header.frame_id="bender/base_link"


    #Movimiento en espacio de joint
    g.moveToJointPosition(joint_names,home)

    rospy.sleep(3.0)
    rospy.loginfo('POSITION: home')

    #Movimiento en espacio cartesiano
    g.moveToPose(spos,"bender/l_wrist_pitch_link")

    rospy.sleep(3.0)
    rospy.loginfo('POSITION: (0.00,0.28,0.59)')
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.3, 0.28, 0.75


    g.moveToPose(spos,"bender/l_wrist_pitch_link")

    rospy.sleep(2.0)

    rospy.loginfo('POSITION: (0.30,0.28,0.75)')
    
    rospy.sleep(1.0)
    rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()