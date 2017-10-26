#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point

def Move(mgi, tip_link, goal):

    result=None
    rospy.loginfo('Starting planning for: [{0}] [{1}] [{2}]'.format(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z))
    result=mgi.moveToPose(goal,tip_link)


    if(result.error_code.val==1):
        rospy.loginfo('Planning Succeeded')
        rospy.sleep(1.0)
        
    else:
        rospy.logerr('Planning Failed')
        rospy.logerr('Error code: {0}'.format(result.error_code.val))

        rospy.sleep(1.0)




def main():
        
    mgi = MoveGroupInterface("nova_arm", "r_arm_support_link",None,False)  #"bender/l_shoulder_pitch_link"
    tip_link="r_wrist_pitch_link"
    joint_names=['r_shoulder_roll_joint','r_shoulder_pitch_joint',
                'r_elbow_pitch_joint','r_elbow_roll_joint',
                'r_wrist_roll_joint','r_wrist_pitch_joint']

    rospy.loginfo('INICIA TEST')
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    #----------------------------------------------------------------

    goal=PoseStamped()
    #Position 
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 0.2, 0.1, 0.1
    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = -0.0148,-0.733 , -0.0404, 0.678 #-0.0148,-0.733 , 0.0, 0.678
    goal.header.frame_id="r_arm_support_link"


    #Movimiento en espacio de joint
    mgi.moveToJointPosition(joint_names,home)
    rospy.loginfo('POSITION: home')
    rospy.sleep(0.5)

    #Movimiento en espacio cartesiano
    Move(mgi,tip_link,goal)
    rospy.sleep(0.5)

    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 0.3, 0.28, 0.75
    Move(mgi,tip_link,goal)
    rospy.sleep(0.5)

    rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('Nova_set_pos')
    main()