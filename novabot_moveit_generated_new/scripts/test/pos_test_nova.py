#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Sebastian Parra'
__email__ = 'sebastian.parra.flores@gmail.com'

import rospy
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point



def main():
    
    
    g = MoveGroupInterface("nova_arm", "r_arm_support_link",None,False)  #"bender/l_shoulder_pitch_link"

    joint_names=['r_shoulder_roll_joint','r_shoulder_pitch_joint',
                'r_elbow_pitch_joint','r_elbow_roll_joint',
                'r_wrist_roll_joint','r_wrist_pitch_joint']
    
    rospy.loginfo('INICIA TEST')
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    #----------------------------------------------------------------

    spos=PoseStamped()
    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z =  0.2, 0.1, 0.1

    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = -0.0148,-0.733 , -0.0404, 0.678

    spos.header.frame_id="r_arm_support_link"


    #Movimiento en espacio de joint
    g.moveToJointPosition(joint_names,home)

    rospy.loginfo('POSITION: home')

    #Movimiento en espacio cartesiano
    result = g.moveToPose(spos,"r_wrist_pitch_link")
    print(result.error_code.val)


    rospy.loginfo('POSITION: (0.00,0.28,0.59)')
    

    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.3, 0.28, 0.75

    result = g.moveToPose(spos,"r_wrist_pitch_link")
    print(result.error_code.val)

    rospy.loginfo('POSITION: (0.30,0.28,0.75)')
    
    rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()