#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_python import MoveGroupInterface
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
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

if __name__=='__main__':

    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()
    l_arm = MoveGroupCommander("l_arm")
    rospy.loginfo(l_arm.get_end_effector_link())
    #l_arm.set_end_effector_link("l_wrist_pitch_link")
    #l_gripper = MoveGroupCommander("l_gripper")
    rospy.sleep(1)

    grasps = []

    g = Grasp()
    g.id = "test"

    spos=PoseStamped()
    spos.header.frame_id="bender/base_link"
    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.2, 0.0, 0.9
    #Z min = 0.75
    #Z max = 1.23
    #X min = 0.12
    #premanip = 0.12, 0.0, 0.75
    quate = get_quaternion_from_euler(math.pi/2, -math.pi, -math.pi/2)
    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    #spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = -0.0148,-0.733 , -0.0404, 0.678
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = quate[0], quate[1], quate[2], quate[3]

    g.grasp_pose = spos

    g.pre_grasp_approach.direction.header.frame_id = "bender/base_link"
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0.0
    g.pre_grasp_approach.direction.vector.z = 0.0
    g.pre_grasp_approach.min_distance = 0.001
    g.pre_grasp_approach.desired_distance = 0.1

    g.pre_grasp_posture.header.frame_id = "bender/base_link"
    g.pre_grasp_posture.joint_names = ["l_int_finger_joint", "l_ext_finger_joint"]
   
    pos = JointTrajectoryPoint()
    pos.positions.append(0.25)
    pos.positions.append(0.25)
   
    g.pre_grasp_posture.points.append(pos)
   
    # set the grasp posture
    g.grasp_posture.header.frame_id = "bender/base_link"
    g.grasp_posture.joint_names = ["l_int_finger_joint", "l_ext_finger_joint"]

    pos = JointTrajectoryPoint()
    pos.positions.append(0.0)
    pos.positions.append(0.0)
    pos.effort.append(0.0)
   
    g.grasp_posture.points.append(pos)

    # set the post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "bender/base_link"
    g.post_grasp_retreat.direction.vector.x = 0.0
    g.post_grasp_retreat.direction.vector.y = 0.0
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.desired_distance = 0.25
    g.post_grasp_retreat.min_distance = 0.01

    g.allowed_touch_objects = [""]

    g.max_contact_force = 0

    # append the grasp to the list of grasps
    grasps.append(g)
    rospy.sleep(2)

    result = None
    n_attempts = 0
    max_pick_attempts = 10    
    # Repeat until we succeed or run out of attempts
    while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
        n_attempts += 1
        rospy.loginfo("Pick attempt: " +  str(n_attempts))
        result = l_arm.pick("part", grasps)
        rospy.sleep(0.2)
    # pick the object
    # robot.l_arm.pick("part", grasps)

    rospy.spin()
    roscpp_shutdown()