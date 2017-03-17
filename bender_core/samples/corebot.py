#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
__author__ = 'Matías Pavez'
__email__  = 'matias.pavez@ing.uchile.cl'

import rospy
from bender_core import robot_factory

if __name__ == "__main__":

    rospy.init_node("core_bot_example")

    print "--------------------------------------------------------------------"
    robot = robot_factory.build()
    print "--------------------------------------------------------------------"
    robot.check()
    robot.setup()
    print "--------------------------------------------------------------------"
    print robot
    print "--------------------------------------------------------------------"    
    robot.start()
    print "--------------------------------------------------------------------"
    robot.pause()
    print "--------------------------------------------------------------------"



