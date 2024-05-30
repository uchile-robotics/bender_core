#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
from bender_arm_control.checkpoint_planner import *
from bender_arm_control.arm_commander import Limb,Arm

def main():
  
    
    CheckpointPlanner()
    #asd = CheckpointPlanner()
    #print asd.go('home','pre_1')

    l_limb = Limb('l')

    # print asd.go('carry','shelf_1')
    l_limb.arm.move_joint_blind('carry','shelf_3')
    l_limb.arm.wait()

    # l_limb.arm.set_position_named('premanip_1')  
    # l_limb.arm.set_position_named('carrypos')
    # l_limb.arm.set_position_named('shelf')  


    # l_limb.move_joint([[0.262, 0.319, 0.665, 1.585, -0.327, -0.112]])
    # l_limb.move_joint_blind('pre_1','pre_2')
    # l_limb.wait()
    # l_limb.move_joint_blind('pre_2','pre_1')
    # l_limb.wait()
    # l_limb.move_joint_blind('pre_1','home')
    # l_limb.wait()

if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()