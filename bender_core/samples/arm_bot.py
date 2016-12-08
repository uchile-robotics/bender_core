#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Rodrigo Muñoz"

import rospy
import tf
# Robot hardware
from bender_core import robot_factory
from bender_core.robot_skill import Log

"""
Example of use of basic arm and gripper interfaces

With the real robot you should launch this before:
$ roslaunch bender_fieldbus l_port.launch
$ roslaunch bender_fieldbus r_port.launch
$ roslaunch bender_arm l_arm.launch
$ roslaunch bender_arm r_arm.launch
$ roslaunch bender_gripper l_gripper.launch
$ roslaunch bender_gripper r_gripper.launch

With gazebo simulation:
$ roslaunch bender_gazebo bender.launch
"""
if __name__ == "__main__":
    rospy.init_node("manipulator_bot_example")
    robot = robot_factory.build(["l_arm","r_arm","l_gripper","r_gripper"])
    # Run check
    if not robot.check():
        raise RuntimeError("Required skills don't available")
    # Run setup
    if not robot.setup():
        raise RuntimeError("Robot setup failed")
    # Set logger for l_gripper for only display error messages
    robot.l_gripper.set_log_level(Log.ERROR)
    # Display joint names in order
    rospy.loginfo(robot.l_arm.get_joint_names())
    rospy.loginfo(robot.r_arm.get_joint_names())
    # Use shared tf listener (from any skill)
    tf_listener = robot.l_arm.get_context().get_tf_listener()
    rospy.sleep(0.1) # Wait for tf
    try:
        (trans, rot) = tf_listener.lookupTransform(
            "bender/base_link", "bender/r_wrist_pitch_link", rospy.Time(0.0))
        rospy.loginfo("Translation: {}".format(trans))
        rospy.loginfo("Rotation: {}".format(rot))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("tf Exception")
    # Get joint states (from any skill)
    joint_state = robot.r_gripper.get_context().get_joint_state()
    # Send arm goal and wait
    robot.l_arm.send_joint_goal([0.0,0.0,0.0,1.2,0.0,0.0], interval=2.0)
    robot.r_arm.send_joint_goal([0.0,0.0,0.0,1.2,0.0,0.0], interval=2.0)
    robot.l_arm.wait_for_motion_done()
    robot.r_arm.wait_for_motion_done()
    rospy.loginfo(robot.r_arm.get_result())
    # Open grippers
    robot.l_gripper.open()
    robot.r_gripper.open()
    rospy.loginfo(robot.l_gripper.get_result())
    # Closing grippers
    robot.l_gripper.close()
    robot.r_gripper.close()
    rospy.loginfo(robot.r_gripper.get_result())
    # Send arm goal and wait
    robot.l_arm.send_joint_goal([0.0,0.0,0.0,0.0,0.0,0.0], interval=2.0)
    robot.r_arm.send_joint_goal([0.0,0.0,0.0,0.0,0.0,0.0], interval=2.0)
    robot.l_arm.wait_for_motion_done()
    robot.r_arm.wait_for_motion_done()
    # Shutdown robot
    robot.shutdown()