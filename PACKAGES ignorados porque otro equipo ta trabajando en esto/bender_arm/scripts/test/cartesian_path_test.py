#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import sys
import rospy
import math
import copy
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
    spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.45, 0.0, 1.0

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
    robot.l_arm.set_goal_tolerance(0.005)
    #g = MoveGroupInterface("l_arm", "bender/base_link",None,False)  #"bender/l_shoulder_pitch_link"
    gripper = MoveGroupInterface("l_gripper", "bender/base_link", None, False)
    scene = PlanningSceneInterface()
    scene.remove_world_object("part")
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.5
    p.pose.position.y = 0.0
    p.pose.position.z = 0.8
    p.pose.orientation.w = 1.0
    #scene.add_box("part", p, (0.05, 0.05, 0.3))
    
    # joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      # 'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      # 'l_wrist_pitch_joint']

    # rospy.loginfo(robot.l_arm.get_end_effector_link())
    # robot.l_arm.set_end_effector_link("bender/l_grasp_link")
    # rospy.loginfo(robot.l_arm.get_end_effector_link())
    # rospy.loginfo(robot.l_arm.get_pose_reference_frame())
    # rospy.loginfo(robot.l_arm.get_current_pose())
    rospy.sleep(1)

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

    rospy.sleep(1.0)
    rospy.loginfo('POSITION: {}'.format(spos))

    #gripper.moveToJointPosition(["l_int_finger_joint","l_ext_finger_joint"],[0.6, 0.6])
    #rospy.sleep(3.0)
    #spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.5, 0.0, 1.0
    #pub.publish(spos)


    #g.moveToPose(spos,"bender/l_wrist_pitch_link")

    #rospy.sleep(2.0)

    #rospy.loginfo('POSITION: (0.4, 0.0, 1.1)')
    
    #rospy.sleep(1.0)
    scale = 1
    waypoints = []
    wpose = robot.l_arm.get_current_pose().pose
    wpose.position.x += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = robot.l_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

    robot.l_arm.execute(plan, wait=True)

    #rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()