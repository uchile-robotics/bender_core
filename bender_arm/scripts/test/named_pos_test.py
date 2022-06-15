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
    
    
    g = MoveGroupInterface("l_arm", "bender/base_link",None,False)

    joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      'l_wrist_pitch_joint']
    
    rospy.loginfo('INICIA TEST')

    l_arm_home =[0.0,0.0,0.0,0.0,0.0,0.0]
    l_arm_middle=[1.8, 0.0, -0.35, 0.1, 0.0, 0.4]
    l_arm_premanip_1=[-0.89, 0.129, -0.066, 1.674, -0.041, 1.053]
    l_arm_premanip_2=[-0.23, 0.129, -0.102, 1.782, -0.031, 0.997]
    

    #INSERTAR NOMBRE DE POSICION
    named_position="sdasd"
    #############################################################
    named_angles=[0.0,0.0,0.0,0.0,0.0,0.0]
    #solucion temporal, estoy buscando como lo implementa el movegroup de cpp para hacer lo mismo y sacarlo del SRDF
    if named_position=="home":
    	named_angles=l_arm_home
    else:
    	if named_position=="middle":
    		named_angles=l_arm_middle
    	else:
    		if named_position=="premanip_1":
    			named_angles=l_arm_premanip_1
    		else:
    			if named_position=="premanip_2":
    				named_angles=l_arm_premanip_2
    			else:
    				named_position="home"
    				rospy.logerr('ERROR! Unknown position')
    
    #############################################################

    #g.moveToJointPosition(joint_names,l_arm_home)
    #rospy.sleep(2.0)
    rospy.sleep(1.0)
    g.moveToJointPosition(joint_names,named_angles)
    rospy.sleep(2.0)
    pos = "POSITION: " + named_position
    rospy.loginfo(pos)

    rospy.sleep(1.0)
    rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()