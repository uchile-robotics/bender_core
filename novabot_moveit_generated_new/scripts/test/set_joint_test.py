#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point



def Move(mgi, joint_names, goal):

	result=None
	rospy.loginfo('Starting planning for: [{0}] [{1}] [{2}] [{3}] [{4}] [{5}]'.format(goal[0],goal[1],goal[2],goal[3],goal[4],goal[5]))
	result=mgi.moveToJointPosition(joint_names,goal)

	if(result.error_code.val==1):
		rospy.loginfo('Planning Succeeded')
		rospy.sleep(1.0)
		
	else:
		rospy.loginfo('Planning Failed')
		rospy.sleep(1.0)




#TEST para movimiento con skill en espacio de joint
def main():

	mgi = MoveGroupInterface("nova_arm", "r_arm_support_link",None,False)

	joint_names=['r_shoulder_roll_joint','r_shoulder_pitch_joint',
				'r_elbow_pitch_joint','r_elbow_roll_joint',
				'r_wrist_roll_joint','r_wrist_pitch_joint']

	rospy.loginfo('INICIA TEST')
	home =[0.0,0.0,0.0,0.0,0.0,0.0]
	pos_1=[-0.9, 0.5, 0.0, 0.0,0.0,0.0]
	pos_2=[0.373, 0.427, 0.588, 1.770, -0.572, 0.256]
	pre_1=[-0.2, -0.5, 0.5, 0.0, 0.0, 0.2]

	#Movimiento en espacio de joint

	Move(mgi,joint_names,home)
	Move(mgi,joint_names,pre_1)
	Move(mgi,joint_names,pos_1)
	Move(mgi,joint_names,pos_2)
	rospy.loginfo('TERMINA TEST')
	################################


if __name__ == '__main__':
	rospy.init_node('novabot_move_test')
	main()