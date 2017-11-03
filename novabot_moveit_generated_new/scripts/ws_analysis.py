#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
import csv
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point

def my_range(start, end, step):
    while start <= end:
        yield start
        start += step

def Move(mgi, tip_link, goal):

    result=None
    rospy.loginfo('Starting planning for: [{0}] [{1}] [{2}]'.format(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z))
    result=mgi.moveToPose(goal,tip_link)


    if(result.error_code.val==1):
        rospy.loginfo('Planning Succeeded')
        return 1
        
    else:
        rospy.logerr('Planning Failed')
        rospy.logerr('Error code: {0}'.format(result.error_code.val))
        return result.error_code.val#-1




def main():
        
    mgi = MoveGroupInterface("nova_arm", "r_arm_support_link",None,True)  #"bender/l_shoulder_pitch_link"
    tip_link="r_wrist_pitch_link"
    joint_names=['r_shoulder_roll_joint','r_shoulder_pitch_joint',
                'r_elbow_pitch_joint','r_elbow_roll_joint',
                'r_wrist_roll_joint','r_wrist_pitch_joint']

    rospy.loginfo('STARTING TEST')
    count=0
    succ_plan=0
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    mgi.moveToJointPosition(joint_names,home)
    rospy.loginfo('POSITION: home')
    rospy.sleep(0.5)
    #---------------------------------------------------------------------------------------------------------------------------------------------------

    goal=PoseStamped()
    #Position 
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 0.2, 0.1, 0.1
    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = 0.05726, 0.5419 , -0.66364, 0.51251 #-0.0148,-0.733 , 0.0, 0.678
    goal.header.frame_id="r_arm_support_link"
    #---------------------------------------------------------------------------------------------------------------------------------------------------
    #------------------------------------------------Iterarion Params-----------------------------------------------------------------------------------
    dl=0.01 #precision en metros (0.03)
    Xmin=0.0  #-0.05
    Xmax=0.8  #0.8
    Ymin=-0.4 #-0.4
    Ymax=0.4  #0.05
    Zmin=0.0  #0.55
    Zmax=1.2  #0.85  
    # ~17 horas worst case scenario
    #---------------------------------------------------------------------------------------------------------------------------------------------------
    workspace = [] #matrix for saving coordinates and error codes
    for i in my_range(Xmin,Xmax,dl):
        for j in my_range(Ymin,Ymax,dl):
            for k in my_range(Zmin,Zmax,dl):
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = i,j,k
                error_code = Move(mgi,tip_link,goal)
                if(error_code == 1):#Move(mgi,tip_link,goal)==1):
                    succ_plan+=1
                count +=1
                workspace.append([error_code,i,j,k])

    percentage=(succ_plan/count)*100
    rospy.loginfo('succeeded plannings: {0}/{1}  ({2}%)'.format(succ_plan,count,percentage))
    rospy.loginfo('TEST ENDED')
    with open('workspace.csv','wb') as f:
        writer = csv.writer(f)
        writer.writerows(workspace)
    ################################

if __name__ == '__main__':
    rospy.init_node('Nova_ws_analysis')
    main()