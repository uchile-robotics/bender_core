#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

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

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

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

def wait_for_update(box_name):
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

def main():
    
    pub = rospy.Publisher('obj_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    joint_state_topic = ['joint_states:=/bender/joint_states']
    roscpp_initialize(joint_state_topic)
    roscpp_initialize(sys.argv)

    rospy.loginfo('INICIA TEST')
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    #----------------------------------------------------------------

    spos=PoseStamped()


    spos.header.frame_id="bender/base_link"
    #Position 
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.35, 0.0, 0.8

    #Z min = 0.75
    #Z max = 1.23

    #X min = 0.12

    #premanip = 0.12, 0.0, 0.75

    # quate = get_quaternion_from_euler(math.pi/2, -math.pi, -math.pi/2)
    quate = get_quaternion_from_euler(0, 0, 0)
    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    #spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = -0.0148,-0.733 , -0.0404, 0.678
    spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = quate[0], quate[1], quate[2], quate[3]

    
    robot = RobotCommander()
    #g = MoveGroupInterface("l_arm", "bender/base_link",None,False)  #"bender/l_shoulder_pitch_link"
    grasping_group = 'l_gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    gripper = MoveGroupInterface("l_gripper", "bender/base_link", None, False)
        
    scene = PlanningSceneInterface()
    scene.remove_attached_object("bender/l_grasp_link", name="part")
    scene.remove_world_object("part")
    rospy.sleep(1.0)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.5
    p.pose.position.y = 0.0
    p.pose.position.z = 0.8
    p.pose.orientation.w = 1.0
    scene.add_box("part", p, (0.05, 0.05, 0.3))

    # joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      # 'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      # 'l_wrist_pitch_joint']

    # rospy.loginfo(robot.l_arm.get_end_effector_link())
    # robot.l_arm.set_end_effector_link("bender/l_grasp_link")
    # rospy.loginfo(robot.l_arm.get_end_effector_link())
    # rospy.loginfo(robot.l_arm.get_pose_reference_frame())
    # rospy.loginfo(robot.l_arm.get_current_pose())
    rospy.sleep(3)

    #robot.l_arm.set_goal

    #Movimiento en espacio de joint
    #g.moveToJointPosition(joint_names,home)

    #rospy.sleep(3.0)
    #rospy.loginfo('POSITION: home')

    #Movimiento en espacio cartesiano
    pub.publish(spos)
    robot.l_arm.set_pose_target(spos)
    robot.l_arm.go()
    # g.moveToPose(spos,"bender/l_grasp_link")

    rospy.sleep(3.0)
    rospy.loginfo('POSITION: {}'.format(spos))

    gripper.moveToJointPosition(["l_int_finger_joint","l_ext_finger_joint"],[0.6, 0.6])
    rospy.sleep(3.0)

    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.5, 0.0, 0.8
    pub.publish(spos)
    robot.l_arm.set_pose_target(spos)
    robot.l_arm.go()
    rospy.sleep(1.0)
    rospy.loginfo('POSITION: {}'.format(spos))

    scene.attach_box("bender/l_grasp_link", "part", touch_links=touch_links)

    gripper.moveToJointPosition(["l_int_finger_joint","l_ext_finger_joint"],[0.0, 0.0])
    rospy.sleep(3.0)

    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.5, 0.0, 0.9
    pub.publish(spos)
    robot.l_arm.set_pose_target(spos)
    robot.l_arm.go()
    rospy.sleep(3.0)
    rospy.loginfo('POSITION: {}'.format(spos))

    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.35, 0.0, 0.8
    pub.publish(spos)
    robot.l_arm.set_pose_target(spos)
    robot.l_arm.go()
    rospy.sleep(3.0)
    rospy.loginfo('POSITION: {}'.format(spos))



    #g.moveToPose(spos,"bender/l_wrist_pitch_link")

    #rospy.sleep(2.0)

    #rospy.loginfo('POSITION: (0.4, 0.0, 1.1)')
    
    #rospy.sleep(1.0)
    #rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()