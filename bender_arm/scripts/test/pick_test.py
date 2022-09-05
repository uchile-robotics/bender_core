#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Felipe Valenzuela'
__email__ = 'felipeandres.valenzuelar@gmail.com'

import sys
import rospy
import math
import numpy as np
from moveit_python import MoveGroupInterface
from moveit_commander import (
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

if __name__ == "__main__":
    joint_state_topic = ['joint_states:=/bender/joint_states']
    roscpp_initialize(joint_state_topic)
    rospy.init_node("moveit_py_demo", anonymous=True)
    roscpp_initialize(sys.argv)

    pub = rospy.Publisher('obj_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # create the scene and the robot commander
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.loginfo(robot.l_arm.get_end_effector_link()) # verify end effector link (should be: bender/l_grasp_link)
    rospy.loginfo(robot.l_arm.get_current_pose())   # just to get an idea of the pose
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.7
    p.pose.position.y = -0.4
    p.pose.position.z = 1.15
    p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.275
    #scene.add_box("table", p, (0.5, 1.5, 0.75))

    p.pose.position.x = 0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 1.0
    scene.add_box("part", p, (0.05, 0.05, 0.3))

    grasping_group = 'l_gripper'
    touch_links = robot.get_link_names(group=grasping_group)

    # Creation of grasp holder
    grasps = []

    g = Grasp()
    g.id = "test"


    spos=PoseStamped()
    spos.header.frame_id="bender/base_link"

    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.6, 0.0, 1.0 # Same as "part"
    #Orientation
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
    pub.publish(spos) # Publish the pose to topic "obj_pose" to see in rviz
    g.grasp_pose = spos # sets the described pose in the Grasp msg


    g.pre_grasp_approach.direction.header.frame_id = "bender/base_link"
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0.0
    g.pre_grasp_approach.direction.vector.z = 0.0
    g.pre_grasp_approach.min_distance = 0.001
    g.pre_grasp_approach.desired_distance = 0.14

    # set the pre-grasp posture
    g.pre_grasp_posture.header.frame_id = "bender/base_link"
    g.pre_grasp_posture.joint_names = ["l_int_finger_joint", "l_ext_finger_joint"]
   
    pg_pos = JointTrajectoryPoint()
    pg_pos.positions = [0.5, 0.5]
   
    g.pre_grasp_posture.points.append(pg_pos)
   
    # set the grasp posture
    g.grasp_posture.header.frame_id = "bender/base_link"
    g.grasp_posture.joint_names = ["l_int_finger_joint", "l_ext_finger_joint"]

    g_pos = JointTrajectoryPoint()
    g_pos.positions = [0.0, 0.0]
    #g_pos.effort.append(0.0)
   
    g.grasp_posture.points.append(g_pos)

    # set the post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "bender/base_link"
    g.post_grasp_retreat.direction.vector.x = 0.0
    g.post_grasp_retreat.direction.vector.y = 0.0
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.desired_distance = 0.1
    g.post_grasp_retreat.min_distance = 0.01

    g.allowed_touch_objects = ["part"]
    # append the grasp to the list of grasps
    grasps.append(g)
    rospy.sleep(1)

    # pick an object
    # scene.attach_box("bender/l_grasp_link", "part", touch_links=touch_links)
    #robot.l_arm.set_support_surface_name("table")
    robot.l_arm.pick("part",grasps)
    # robot.l_arm.pick("part")


    rospy.spin()
    roscpp_shutdown()