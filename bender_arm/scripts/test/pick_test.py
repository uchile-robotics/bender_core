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
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_matrix, quaternion_from_euler
from gpd_ros.msg import GraspConfigList, GraspConfig
import tf2_geometry_msgs
import copy
from sensor_msgs.msg import PointCloud, Image
from geometry_msgs.msg import Point32
from uchile_msgs.msg import PrimitiveWithPose

class Manipulator():
    def __init__(self):
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.gripper = MoveGroupInterface("l_gripper", "bender/base_link", None, False)
        #self.eef_link = self.robot.l_arm.get_end_effector_link()
        self.robot.l_arm.set_end_effector_link("bender/l_grasp_link")
        self.robot.l_arm.set_planning_time(2)
        self.eef_link = "bender/l_grasp_link"
        self.grasping_group = 'l_gripper'
        self.finger_links = self.robot.get_link_names(group=self.grasping_group)
        #self.grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp)
        self.pose_pub = rospy.Publisher('grasp_pose', PoseStamped, queue_size=10)
        self.post_pose_pub = rospy.Publisher('post_grasp_pose', PoseStamped, queue_size=10)
        self.grasp_pub = rospy.Publisher('grasp', GraspConfigList, queue_size=10)
        self.rate = rospy.Rate(10) # 10 hz

    def grasp(self):
        rospy.loginfo("Robot ready. Waiting for GPD output...")
        msg = rospy.wait_for_message(
            "/detect_grasps/clustered_grasps", GraspConfigList)
        rospy.loginfo("Received {} grasps.".format(len(msg.grasps)))
        grasps = self.GPDtoMoveItGrasp(msg)
        rospy.loginfo("Trying {} selected grasps.".format(len(grasps)))
        self.go_to_pregrasp()
        self.pick(grasps, cartesian=False)

        #for grasp in grasps:
        #    gg = grasp.grasp_pose
        #    self.pose_pub.publish(gg)
        #    self.robot.l_arm.set_pose_target(grasp.grasp_pose)
        #    plan = self.robot.l_arm.plan()
        #    if not plan.joint_trajectory.points:
        #        print("Plan Failed")
        #    else:
        #        print("Plan Found!")
            #while not rospy.is_shutdown():
            #   self.pose_pub.publish(gg)self.pose_pub.publish(gg)
            #    self.grasp_pub.publish(msg)
            #    self.rate.sleep()

        #self.robot.l_arm.pick("part",grasps)
        rospy.loginfo("Grasp Finished")

    def grasp_object(self):
        rospy.loginfo('Robot ready. Waiting for object to be published on "/obj" topic...')
        msg = rospy.wait_for_message("/obj", PrimitiveWithPose)
        grasps = self.grasps_from_primitive(msg)
        self.post_pose_pub.publish(grasps[0].grasp_pose)
        rospy.loginfo("Trying {} selected grasps.".format(len(grasps)))
        self.go_to_pregrasp()
        self.pick(grasps, cartesian=True)
        rospy.loginfo("Grasp Finished")

    def grasp_cylinder(self):
        rospy.loginfo("Robot ready. Waiting for object pose...")
        msg = rospy.wait_for_message(
            "/obj_pos", PoseStamped)
        grasps = self.grasps_from_pose(msg)
        rospy.loginfo("Trying {} selected grasps.".format(len(grasps)))
        self.go_to_pregrasp()
        self.pick(grasps, cartesian=True)
        rospy.loginfo("Grasp Finished")


    def try_pose(self, pose, feedback=False):
        #Try Grasp Pose
        self.robot.l_arm.set_pose_target(pose)
        plan = self.robot.l_arm.plan()
        if not plan.joint_trajectory.points:
            if feedback:
                rospy.loginfo("Grasp Plan Failed :c")
            return False
        else:
            if feedback:
                rospy.loginfo("Grasp Plan Found!")
            return True

    def grasps_from_primitive(self, primitive):
        
        g = Grasp()

        if primitive.shape.type == 3: # CYLINDER

            #Cylinder Spawn
            self.post_pose_pub.publish(primitive.pose)
            self.new_object(primitive.pose, primitive.shape)

            #Parameters
            frame_id = primitive.pose.header.frame_id

            p_height = primitive.shape.dimensions[0]
            p_radius = primitive.shape.dimensions[1]

            grasp_width = 0.5
            finger_joints = ["l_int_finger_joint", "l_ext_finger_joint"]

            #Grasp Config
            g.grasp_pose = primitive.pose
            g.grasp_pose.pose.position.x = primitive.pose.pose.position.x - p_radius - 0.03

            #Grasp Posture Config
            g.grasp_posture.header.frame_id = frame_id
            g.grasp_posture.joint_names = finger_joints
            g_pos = JointTrajectoryPoint()
            g_pos.positions = [0.0, 0.0]
            g_pos.effort.append(5.0)
            g.grasp_posture.points.append(g_pos)

            #Pre-Grasp Posture Config
            g.pre_grasp_posture.header.frame_id = frame_id
            g.pre_grasp_posture.joint_names = finger_joints
            pg_pos = JointTrajectoryPoint()
            point = grasp_width/2
            pg_pos.positions = [0.5, 0.5]
            g.pre_grasp_posture.points.append(pg_pos)

            #Pre-Grasp Config
            g.pre_grasp_approach.direction.header.frame_id = frame_id
            g.pre_grasp_approach.direction.vector.x = 1.0
            g.pre_grasp_approach.direction.vector.y = 0.0
            g.pre_grasp_approach.direction.vector.z = 0.0
            g.pre_grasp_approach.min_distance = 0.08
            g.pre_grasp_approach.desired_distance = 0.2

            #Post Grasp Config
            g.post_grasp_retreat.direction.vector.z = 1.0
            g.post_grasp_retreat.min_distance = 0.03
            g.post_grasp_retreat.desired_distance = 0.05
            g.post_grasp_retreat.direction.header.frame_id = frame_id

        return [g]

    def grasps_from_pose(self, pose):
        
        g = Grasp()

        g.grasp_pose = pose

        #Parameters
        frame_id = pose.header.frame_id
        grasp_width = 0.5
        finger_joints = ["l_int_finger_joint", "l_ext_finger_joint"]

        #Grasp Posture Config
        g.grasp_posture.header.frame_id = frame_id
        g.grasp_posture.joint_names = finger_joints
        g_pos = JointTrajectoryPoint()
        g_pos.positions = [0.0, 0.0]
        g_pos.effort.append(5.0)
        g.grasp_posture.points.append(g_pos)

        #Pre-Grasp Posture Config
        g.pre_grasp_posture.header.frame_id = frame_id
        g.pre_grasp_posture.joint_names = finger_joints
        pg_pos = JointTrajectoryPoint()
        point = grasp_width/2
        pg_pos.positions = [0.5, 0.5]
        g.pre_grasp_posture.points.append(pg_pos)

        #Pre-Grasp Config
        g.pre_grasp_approach.direction.header.frame_id = frame_id
        g.pre_grasp_approach.direction.vector.x = 1.0
        g.pre_grasp_approach.direction.vector.y = 0.0
        g.pre_grasp_approach.direction.vector.z = 0.0
        g.pre_grasp_approach.min_distance = 0.08
        g.pre_grasp_approach.desired_distance = 0.2

        #Post Grasp Config
        g.post_grasp_retreat.direction.vector.z = 1.0
        g.post_grasp_retreat.min_distance = 0.03
        g.post_grasp_retreat.desired_distance = 0.05
        g.post_grasp_retreat.direction.header.frame_id = frame_id

        return [g]



    def GPDtoMoveItGrasp(self, GraspCfgList, N=None, force_plannar=False):
        
        frame_id = GraspCfgList.header.frame_id
        if N is None:
            GCfgL = GraspCfgList.grasps
        else:
            GCfgL = GraspCfgList.grasps[:N]

        grasps = []

        for grasp in GCfgL:
            while True:
                #Parameters
                finger_joints = ["l_int_finger_joint", "l_ext_finger_joint"]
                grasp_width = grasp.width.data

                #Grasp Object Created
                g = Grasp()

                #Grasp Pose Config
                gpose = self.gpd_grasp_to_pose(grasp, frame_id)

                if not gpose:
                    break
                else:
                    g.grasp_pose = gpose

                #Grasp Posture Config
                g.grasp_posture.header.frame_id = frame_id
                g.grasp_posture.joint_names = finger_joints
                g_pos = JointTrajectoryPoint()
                g_pos.positions = [0.0, 0.0]
                g_pos.effort.append(5.0)
                g.grasp_posture.points.append(g_pos)

                #Pre-Grasp Posture Config
                g.pre_grasp_posture.header.frame_id = frame_id
                g.pre_grasp_posture.joint_names = finger_joints
                pg_pos = JointTrajectoryPoint()
                point = grasp_width/2
                pg_pos.positions = [0.5, 0.5]
                g.pre_grasp_posture.points.append(pg_pos)

                #Grasp Score/Quality
                g.grasp_quality = grasp.score.data
                
                #Pre-Grasp Config
                g.pre_grasp_approach.direction.header.frame_id = frame_id
                g.pre_grasp_approach.direction.vector.x = grasp.approach.x
                g.pre_grasp_approach.direction.vector.y = grasp.approach.y
                g.pre_grasp_approach.direction.vector.z = grasp.approach.z
                g.pre_grasp_approach.min_distance = 0.08
                g.pre_grasp_approach.desired_distance = 0.2

                #Post Grasp Config
                g.post_grasp_retreat.direction.vector.z = 1.0
                g.post_grasp_retreat.min_distance = 0.05
                g.post_grasp_retreat.desired_distance = 0.05
                g.post_grasp_retreat.direction.header.frame_id = frame_id

                grasps.append(g)
                break

        return grasps

    def gpd_grasp_to_pose(self, grasp, frame_id="bender/base_link"):
        p = PoseStamped()
        p.header.frame_id = frame_id

        x_approach,y_approach,z_approach = grasp.approach.x, grasp.approach.y, grasp.approach.z
        x_binormal,y_binormal,z_binormal = grasp.binormal.x, grasp.binormal.y, grasp.binormal.z
        x_axis,y_axis,z_axis = grasp.axis.x, grasp.axis.y, grasp.axis.z

        if not (x_approach >= 0 and z_approach <= 0 and z_axis <= 0): return 0
        #and y_approach <=0

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
    
    def wait_for_update(self, box_name="part", box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

    def clean_scene(self, box_name="part"):
        #Clean object attachment
        self.scene.remove_attached_object(self.eef_link, name=box_name)
        self.wait_for_update(box_name=box_name, box_is_known=True)

        #Remove object from scene
        self.scene.remove_world_object("part")
        self.wait_for_update(box_name=box_name)
        rospy.loginfo("Scene cleaned c:")

    def new_object(self, pose, shape=None, box_name="part"):
        #Create new object from grasp pose
        self.clean_scene(box_name)
        if shape is not None:
            if shape.type == 1: # BOX
                self.scene.add_box(box_name, pose, (0.02, 0.02, 0.02))
            if shape.type == 3: # CYLINDER
                self.scene.add_box(box_name, pose, (shape.dimensions[1]*2, shape.dimensions[1]*2, shape.dimensions[0]))
        self.wait_for_update(box_name=box_name, box_is_known=True)

    def add_table(self, xyz, size, frame_id):
        self.clean_scene("table")
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        p.pose.position.x = xyz[0]
        p.pose.position.y = xyz[1]
        p.pose.position.z = xyz[2]
        self.scene.add_box("table", p, size)
        self.wait_for_update(box_name="table", box_is_known=True)

    def go_to_pregrasp(self):
        spos=PoseStamped()
        spos.header.frame_id="bender/base_link"

        #Position 
        spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = 0.45, 0.25, 0.85 #0.28, 0.0, 0.8
        #Orientation
        spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = 0.0, 0.0, 0.0, 1.0

        self.robot.l_arm.set_pose_target(spos) #Set Pre-grasp 
        self.robot.l_arm.go() #Move Gripper to Pre-Grasp

    def pick(self, grasps, obj_name="part", cartesian=False, create_obj=False):
        for grasp in grasps:
            grasp_done = False
            while True:
                #Get grasp pose
                grasp_pose = PoseStamped()
                grasp_pose = grasp.grasp_pose

                self.pose_pub.publish(grasp_pose)

                #Get the pre-grasp and grasp posture
                gripper_frame_id = grasp.pre_grasp_posture.header.frame_id
                gripper_joint_names = grasp.pre_grasp_posture.joint_names
                gripper_pg_positions = grasp.pre_grasp_posture.points[0].positions
                gripper_g_positions = grasp.grasp_posture.points[0].positions

                
                self.gripper.moveToJointPosition(gripper_joint_names,gripper_pg_positions)

                #Try Grasp Pose
                self.robot.l_arm.set_pose_target(grasp_pose)
                if cartesian:
                    (plan, fraction) = self.robot.l_arm.compute_cartesian_path([grasp_pose.pose], 0.01, 0.0)
                else:
                    plan = self.robot.l_arm.plan()

                if not plan.joint_trajectory.points:
                    rospy.loginfo("Grasp Plan Failed :c ({},{},{})".format(grasp_pose.pose.position.x,
                                                                           grasp_pose.pose.position.y,
                                                                           grasp_pose.pose.position.z))
                    break
                else:
                    rospy.loginfo("Grasp Plan Found! ({},{},{})".format(grasp_pose.pose.position.x,
                                                                        grasp_pose.pose.position.y,
                                                                        grasp_pose.pose.position.z))

                #Get pre-grasp pose from appproach vector
                pre_grasp_pose = PoseStamped()
                pre_grasp_pose.pose.orientation = grasp.grasp_pose.pose.orientation
                pg_frame = grasp.pre_grasp_approach.direction.header.frame_id
                approach_x = grasp.pre_grasp_approach.direction.vector.x
                approach_y = grasp.pre_grasp_approach.direction.vector.y
                approach_z = grasp.pre_grasp_approach.direction.vector.z
                min_d = grasp.pre_grasp_approach.min_distance
                desired_d =grasp.pre_grasp_approach.desired_distance

                pre_grasp_pose.header.frame_id = pg_frame
                pre_grasp_pose.pose.position.x = grasp_pose.pose.position.x - approach_x*desired_d
                pre_grasp_pose.pose.position.y = grasp_pose.pose.position.y - approach_y*desired_d
                pre_grasp_pose.pose.position.z = grasp_pose.pose.position.z - approach_z*desired_d

                #Try Pre-Grasp Pose
                self.robot.l_arm.set_pose_target(pre_grasp_pose)
                if cartesian:
                    (plan, fraction) = self.robot.l_arm.compute_cartesian_path([pre_grasp_pose.pose], 0.01, 0.0)
                else:
                    plan = self.robot.l_arm.plan()

                if not plan.joint_trajectory.points:
                    rospy.loginfo("Pre-Grasp Plan Failed :c")
                    use_pregrasp = False
                else:
                    rospy.loginfo("Pre-Grasp Plan Found!")
                    use_pregrasp = True

                #Get post-grasp pose from retreat vector

                post_grasp_pose = PoseStamped()
                post_grasp_pose.pose.orientation = grasp.grasp_pose.pose.orientation
                retreat_x = grasp.post_grasp_retreat.direction.vector.x
                retreat_y = grasp.post_grasp_retreat.direction.vector.y
                retreat_z = grasp.post_grasp_retreat.direction.vector.z
                post_min_d = grasp.post_grasp_retreat.min_distance
                post_desired_d = grasp.post_grasp_retreat.desired_distance

                post_grasp_pose.header.frame_id = pg_frame
                post_grasp_pose.pose.position.x = grasp_pose.pose.position.x + retreat_x*post_desired_d
                post_grasp_pose.pose.position.y = grasp_pose.pose.position.y + retreat_y*post_desired_d
                post_grasp_pose.pose.position.z = grasp_pose.pose.position.z + retreat_z*post_desired_d

                #Execute Pipeline
                if use_pregrasp:
                    if cartesian:
                        (plan, fraction) = self.robot.l_arm.compute_cartesian_path([pre_grasp_pose.pose], 0.01, 0.0)
                        self.robot.l_arm.execute(plan, wait=True)
                    else:
                        self.robot.l_arm.set_pose_target(pre_grasp_pose) #Set Pre-grasp 
                    self.robot.l_arm.go() #Move Gripper to Pre-Grasp
                    rospy.sleep(0.2)
                    (plan, fraction) = self.robot.l_arm.compute_cartesian_path([grasp_pose.pose], 0.01, 0.0)
                    self.robot.l_arm.execute(plan, wait=True)
                    rospy.sleep(0.2)
                else:
                    self.robot.l_arm.set_pose_target(grasp_pose) #Set Pre-grasp 
                    self.robot.l_arm.go() #Move Gripper to Pre-Grasp
                    rospy.sleep(0.2)
                if create_obj:
                    self.new_object(grasp, obj_name)
                #self.robot.l_arm.set_pose_target(grasp_pose) #Set Grasp 
                #self.robot.l_arm.go() #Move Gripper to Grasp
                rospy.sleep(0.2)
                #ATTACH OBJECT
                self.scene.attach_box(self.eef_link, obj_name, touch_links=self.finger_links)
                self.wait_for_update(obj_name, box_is_known=False, box_is_attached=True)
                rospy.sleep(0.2)
                self.gripper.moveToJointPosition(gripper_joint_names,gripper_g_positions)
                rospy.sleep(0.2)
                #POST GRASP
                #self.robot.l_arm.set_pose_target(post_grasp_pose) #Go to Post Grasp 
                #self.robot.l_arm.go()
                self.post_pose_pub.publish(post_grasp_pose)
                if cartesian:
                    (plan, fraction) = self.robot.l_arm.compute_cartesian_path([post_grasp_pose.pose], 0.01, 0.0)
                else:
                    self.robot.l_arm.set_pose_target(post_grasp_pose) #Set Pre-grasp 
                    self.robot.l_arm.go() #Move Gripper to Pre-Grasp
                self.robot.l_arm.execute(plan, wait=True)
                rospy.sleep(0.2)
                #self.robot.l_arm.set_pose_target(pre_grasp_pose)
                #self.robot.l_arm.go() #Move Gripper to Pre-Grasp Again
                #rospy.sleep(1)
                grasp_done = True
                break
            if grasp_done:
                self.go_to_pregrasp()
                break






if __name__ == "__main__":
    joint_state_topic = ['joint_states:=/bender/joint_states']
    roscpp_initialize(joint_state_topic)
    rospy.init_node("pick_demo", anonymous=True)
    roscpp_initialize(sys.argv)
    rate = rospy.Rate(10) # 10hz
    g = Manipulator()
    g.clean_scene()
    g.add_table([1.05, 0.0, 0.4725],(0.8, 1.5, 0.945),"bender/base_footprint")
    rospy.sleep(1)

    while not rospy.is_shutdown():
        g.grasp_object()

    rospy.spin()
    roscpp_shutdown()