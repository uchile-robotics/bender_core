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
from tf.transformations import euler_from_matrix, quaternion_from_euler
from gpd_ros.msg import GraspConfigList, GraspConfig
import tf2_geometry_msgs
import copy
from sensor_msgs.msg import PointCloud, Image
from geometry_msgs.msg import Point32

class PostGPD():
    def __init__(self):
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        #self.grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp)
        self.pose_pub = rospy.Publisher('new_grasp_pose', PoseStamped, queue_size=10)
        #self.grasp_pub = rospy.Publisher('grasp', GraspConfig, queue_size=10)
        self.rate = rospy.Rate(10) # 10 hz

    def grasp(self):
        rospy.loginfo("Robot ready. Waiting for GPD output...")
        msg = rospy.wait_for_message(
            "grasp", GraspConfigList)

        grasps = self.GPDtoMoveItGrasp(msg)

        for i, grasp in enumerate(grasps):
            gg = grasp.grasp_pose
            #self.robot.l_arm.set_pose_target(grasp.grasp_pose)
            #self.robot.l_arm.plan()
            while not rospy.is_shutdown():
                self.pose_pub.publish(gg)
                #self.grasp_pub.publish(msg.grasps[i])
                self.rate.sleep()

        #self.robot.l_arm.pick("part",grasps)

    def GPDtoMoveItGrasp(self, GraspCfgList, N=None):
        
        frame_id = GraspCfgList.header.frame_id
        if N is None:
            GCfgL = GraspCfgList.grasps
        else:
            GCfgL = GraspCfgList.grasps[:N]

        grasps = []

        for grasp in GCfgL:
            #Parameters
            finger_joints = ["l_int_finger_joint", "l_ext_finger_joint"]

            #Grasp Object Created
            g = Grasp()

            #Grasp Posture Config
            g.grasp_posture.header.frame_id = frame_id
            g.grasp_posture.joint_names = finger_joints
            g_pos = JointTrajectoryPoint()
            g_pos.positions = [0.0, 0.0]
            g_pos.effort.append(0.0)
            g.grasp_posture.points.append(g_pos)

            #Pre-Grasp Posture Config
            g.pre_grasp_posture.header.frame_id = frame_id
            g.pre_grasp_posture.joint_names = finger_joints
            pg_pos = JointTrajectoryPoint()
            pg_pos.positions = [0.5, 0.5]
            g.pre_grasp_posture.points.append(pg_pos)

            #Grasp Score/Quality
            g.grasp_quality = grasp.score.data

            #Grasp Pose Config
            g.grasp_pose = self.gpd_grasp_to_pose(grasp, frame_id)
            
            #Pre-Grasp Config
            g.pre_grasp_approach.direction.header.frame_id = frame_id
            g.pre_grasp_approach.direction.vector.x = grasp.approach.x
            g.pre_grasp_approach.direction.vector.y = grasp.approach.y
            g.pre_grasp_approach.direction.vector.z = grasp.approach.z
            g.pre_grasp_approach.min_distance = 0.08
            g.pre_grasp_approach.desired_distance = 0.1

            #Post Grasp Config
            g.post_grasp_retreat.direction.vector.z = 1.0
            g.post_grasp_retreat.min_distance = 0.13
            g.post_grasp_retreat.desired_distance = 0.15
            g.post_grasp_retreat.direction.header.frame_id = frame_id

            grasps.append(g)

        return grasps

    def gpd_grasp_to_pose(self, grasp, frame_id="bender/base_link"):
        p = PoseStamped()
        p.header.frame_id = frame_id

        x_approach,y_approach,z_approach = grasp.approach.x, grasp.approach.y, grasp.approach.z
        x_binormal,y_binormal,z_binormal = grasp.binormal.x, grasp.binormal.y, grasp.binormal.z
        x_axis,y_axis,z_axis = grasp.axis.x, grasp.axis.y, grasp.axis.z

        R = [[x_approach,-x_binormal,-x_axis], 
             [y_approach,-y_binormal,-y_axis], 
             [z_approach,-z_binormal,-z_axis]]

        euler_ori = euler_from_matrix(R)
        ori = quaternion_from_euler(euler_ori[0],euler_ori[1],euler_ori[2])
        p.pose.orientation.x = ori[0]
        p.pose.orientation.y = ori[1]
        p.pose.orientation.z = ori[2]
        p.pose.orientation.w = ori[3] 

        p.pose.position = grasp.position

        return p






if __name__ == "__main__":
    joint_state_topic = ['joint_states:=/bender/joint_states']
    roscpp_initialize(joint_state_topic)
    rospy.init_node("pick_demo", anonymous=True)
    roscpp_initialize(sys.argv)
    rate = rospy.Rate(10) # 10hz

    g = PostGPD()
    g.grasp()

    rospy.spin()
    roscpp_shutdown()