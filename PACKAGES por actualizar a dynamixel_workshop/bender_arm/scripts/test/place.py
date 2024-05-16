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

def wait_for_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
  start = rospy.get_time()
  seconds = rospy.get_time()
  while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0

    # Test if the box is in the scene.
    # Note that attaching the box will remove it from known_objects
    is_known = box_name in scene.get_known_object_names()

    # Test if we are in the expected state
    if (box_is_attached == is_attached) and (box_is_known == is_known):
      return True

    # Sleep so that we give other threads time on the processor
    rospy.sleep(0.1)
    seconds = rospy.get_time()

  # If we exited the while loop without returning then we timed out
  return False

def place(scene, move_group, gripper_interface, finger_links, obj_name, place_loc):
  #EEF Link
  eef_link = move_group.get_end_effector_link()
  #Get place pose
  place_pose = PoseStamped()
  place_pose = place_loc.place_pose
  #Get pre-place pose from appproach vector
  pre_place_pose = PoseStamped()
  pre_place_pose.pose.orientation = place_pose.pose.orientation
  pp_frame = place_loc.pre_place_approach.direction.header.frame_id
  approach_x = place_loc.pre_place_approach.direction.vector.x
  approach_y = place_loc.pre_place_approach.direction.vector.y
  approach_z = place_loc.pre_place_approach.direction.vector.z
  min_d = place_loc.pre_place_approach.min_distance
  desired_d = place_loc.pre_place_approach.desired_distance
  pre_place_pose.header.frame_id = pp_frame
  pre_place_pose.pose.position.x = place_pose.pose.position.x + approach_x*desired_d
  pre_place_pose.pose.position.y = place_pose.pose.position.y + approach_y*desired_d
  pre_place_pose.pose.position.z = place_pose.pose.position.z + approach_z*desired_d
  #Get post-grasp pose from retreat vector
  post_place_pose = PoseStamped()
  post_place_pose.pose.orientation = place_pose.pose.orientation
  retreat_x = place_loc.post_place_retreat.direction.vector.x
  retreat_y = place_loc.post_place_retreat.direction.vector.y
  retreat_z = place_loc.post_place_retreat.direction.vector.z
  post_min_d = place_loc.post_place_retreat.min_distance
  post_desired_d = place_loc.post_place_retreat.desired_distance
  post_place_pose.header.frame_id = pp_frame
  post_place_pose.pose.position.x = place_pose.pose.position.x + retreat_x*post_desired_d
  post_place_pose.pose.position.y = place_pose.pose.position.y + retreat_y*post_desired_d
  post_place_pose.pose.position.z = place_pose.pose.position.z + retreat_z*post_desired_d
  #Get the pre-grasp and grasp posture
  gripper_frame_id = place_loc.post_place_posture.header.frame_id
  gripper_joint_names = place_loc.post_place_posture.joint_names
  gripper_pp_positions = place_loc.post_place_posture.points[0].positions
  #Execute Pipeline
  move_group.set_pose_target(pre_place_pose) #Set Pre-grasp 
  move_group.go() #Move Gripper to Pre-Grasp
  rospy.sleep(1)

  move_group.set_pose_target(place_pose) #Set Pre-grasp 
  move_group.go() #Move Gripper to Pre-Grasp
  rospy.sleep(1)

  gripper_interface.moveToJointPosition(gripper_joint_names,gripper_pp_positions)
  rospy.sleep(1)
  #(plan, fraction) = move_group.compute_cartesian_path([grasp_pose.pose], 0.01, 0.0)
  #move_group.execute(plan, wait=True)

  scene.remove_attached_object(eef_link, name=obj_name)
  wait_for_update(obj_name, scene, box_is_known=True, box_is_attached=False)

  move_group.set_pose_target(post_place_pose) #Set Grasp 
  move_group.go() #Move Gripper to Grasp
  rospy.sleep(1)




        




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
    eef_link = robot.l_arm.get_end_effector_link()
    l_gripper = MoveGroupInterface("l_gripper", "bender/base_link", None, False)
    grasping_group = 'l_gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    rospy.loginfo(robot.l_arm.get_end_effector_link()) # verify end effector link (should be: bender/l_grasp_link)
    rospy.loginfo(robot.l_arm.get_current_pose())   # just to get an idea of the pose
    rospy.sleep(1)


    # clean the scene
    #scene.remove_attached_object(eef_link, name="part")
    #wait_for_update("part", scene, box_is_known=True, box_is_attached=False)

    #scene.remove_world_object("pole")
    #scene.remove_world_object("table")
    #scene.remove_world_object("part")

    #wait_for_update("part", scene, box_is_known=False, box_is_attached=False)
    # publish a demo scene
    #p = PoseStamped()
    #p.header.frame_id = robot.get_planning_frame()
    #p.pose.position.x = 0.7
    #p.pose.position.y = -0.4
    #p.pose.position.z = 1.15
    #p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.3, 0.1, 1.0))

    #p.pose.position.y = -0.2
    #p.pose.position.z = 0.425
    #scene.add_box("table", p, (0.5, 1.5, 0.85))

    #p.pose.position.x = 0.6
    #p.pose.position.y = 0.0
    #p.pose.position.z = 1.0
    #scene.add_box("part", p, (0.05, 0.05, 0.3))
    #wait_for_update("part", scene, box_is_known=True, box_is_attached=False)

    grasping_group = 'l_gripper'
    touch_links = robot.get_link_names(group=grasping_group)

    p = PlaceLocation()
    p.id = "test"


    spos=PoseStamped()
    spos.header.frame_id="bender/base_link"

    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.7, 0.3, 1.0 # Same as "part"
    #Orientation
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
    pub.publish(spos) # Publish the pose to topic "obj_pose" to see in rviz
    p.place_pose = spos # sets the described pose in the Grasp msg


    p.pre_place_approach.direction.header.frame_id = "bender/base_link"
    p.pre_place_approach.direction.vector.x = 0.0
    p.pre_place_approach.direction.vector.y = 0.0
    p.pre_place_approach.direction.vector.z = -1.0
    p.pre_place_approach.min_distance = 0.001
    p.pre_place_approach.desired_distance = 0.02

    # set the pre-grasp posture
    p.post_place_posture.header.frame_id = "bender/base_link"
    p.post_place_posture.joint_names = ["l_int_finger_joint", "l_ext_finger_joint"]
   
    pp_pos = JointTrajectoryPoint()
    pp_pos.positions = [0.5, 0.5]
   
    p.post_place_posture.points.append(pp_pos)

    # set the post-grasp retreat
    p.post_place_retreat.direction.header.frame_id = "bender/base_link"
    p.post_place_retreat.direction.vector.x = -1.0
    p.post_place_retreat.direction.vector.y = 0.0
    p.post_place_retreat.direction.vector.z = 0.0
    p.post_place_retreat.desired_distance = 0.05
    p.post_place_retreat.min_distance = 0.14

    p.allowed_touch_objects = ["part"]
    # append the grasp to the list of grasps
    #grasps.append(g)
    rospy.sleep(1)

    # pick an object
    # scene.attach_box("bender/l_grasp_link", "part", touch_links=touch_links)
    # robot.l_arm.set_support_surface_name("table")
    # robot.l_arm.set_num_planning_attempts(10)
    # print(robot.l_arm.plan(g.grasp_pose))
    # robot.l_arm.place("part",p)
    # robot.l_arm.pick("part")

    place(scene,robot.l_arm,l_gripper,touch_links,"part",p)

    rospy.spin()
    roscpp_shutdown()
