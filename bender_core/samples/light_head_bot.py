#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Rodrigo Muñoz"

import rospy
import sys
from bender_core.robot import Robot

# hardware
from bender_core.core.light_head import LightHead


def build_robot():
    # Base robot
    robot = Robot("light_head_bot")
    # Add head
    robot.set(LightHead.get_instance())
    if not robot.check():
        raise RuntimeError("Required skills dont available")
    # Run setup
    if not robot.setup():
        raise RuntimeError("Robot setup failed")
    return robot

if __name__ == "__main__":
    rospy.init_node("manipulator_bot_example")
    robot = build_robot()
    # Get arms
    head = robot.get("light_head")
    # Dsiplay joint names in order
    rospy.loginfo(head.get_joint_names())
    rospy.loginfo(head.get_joint_state())
    # Send head goal and wait
    head.home()
    head.wait_for_motion_done()
    # Send head goal and wait
    head.look_at_ground()
    head.wait_for_motion_done()
    # Send head goal and wait
    head.send_joint_goal(yaw=-1.0, pitch=0.3)
    head.wait_for_motion_done()
    head.send_joint_goal(yaw=1.0, pitch=-0.3)
    head.wait_for_motion_done()
    # Shutdown robot
    robot.shutdown()
  