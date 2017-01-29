#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Rodrigo Muñoz"

import rospy
import tf
# Robot hardware
from bender_core import robot_factory
from bender_core.robot_skill import Log

"""
Example for gripper calibration

With the real robot you should launch this before:
$ roslaunch bender_fieldbus r_port.launch
$ roslaunch bender_arm r_arm.launch
$ roslaunch bender_gripper r_gripper.launch
$ roslaunch bender_tf tf_model.launch

With gazebo simulation:
$ roslaunch bender_gazebo bender.launch
"""
if __name__ == "__main__":
    rospy.init_node("manipulator_bot_example")
    robot = robot_factory.build(["r_arm","r_gripper"])
    # Run check
    if not robot.check():
        raise RuntimeError("Required skills don't available")
    # Run setup
    if not robot.setup():
        raise RuntimeError("Robot setup failed")
    # Send arm goal and wait
    robot.r_arm.send_joint_goal([-0.84, 0.0, 0.0, 1.8, 0.0, 1.2 ], interval=2.0)
    robot.r_arm.wait_for_motion_done()
    robot.r_arm.send_joint_goal([ -0.6, 0.0, 0.0, 1.8, 0.0, 0.26], interval=2.0)
    robot.r_arm.wait_for_motion_done()
    # Open gripper
    robot.r_gripper.open()
    rospy.sleep(1.0)
    rospy.loginfo(robot.r_gripper.get_result())
    # Go to grap position
    robot.r_arm.send_joint_goal([-0.01, 0.0, 0.0, 1.34, 0.0, 0.26], interval=2.0)
    robot.r_arm.wait_for_motion_done()
    
    while not rospy.is_shutdown(): 
        # Close gripper
        robot.r_gripper.close()
        rospy.loginfo(robot.r_gripper.get_result())
        rospy.sleep(1.0)
        # Open gripper
        robot.r_gripper.open()
        rospy.loginfo(robot.r_gripper.get_result())
        rospy.sleep(1.0)

    # Shutdown robot
    robot.shutdown()