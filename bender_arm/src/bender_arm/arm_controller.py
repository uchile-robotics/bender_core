#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

# Numpy
import numpy as np
# Thread
from threading import Thread
# ROS
import rospy
from dynamixel_controllers.joint_trajectory_action_controller import JointTrajectoryActionController
from sensor_msgs.msg import JointState as SensorJointState
from std_msgs.msg import String

class ArmController(JointTrajectoryActionController):
    def __init__(self, controller_namespace, controllers):
        JointTrajectoryActionController.__init__(self, controller_namespace, controllers)
        # joint_states base msg
        self.sensor_joint_state = SensorJointState()
        self.sensor_joint_state.name = self.joint_names
        self.sensor_joint_state.position = [0.0]*self.num_joints
        self.sensor_joint_state.velocity = [0.0]*self.num_joints
        self.sensor_joint_state.effort = [0.0]*self.num_joints
        # Gain scheduling
        self.gain_scheduling = GainScheduling(threshold = 0.5)
        #self.checkpoint = Checkpoint(controller_namespace=controller_namespace)
      
    def initialize(self):
        # Topic name for publish joint_states
        self.joint_states_topic = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/joint_states_topic', 'joint_states')
        # Parametro rate para /joint_states
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/state_update_rate', 50)

        # Gain scheduling params
        op_params = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/operation_points')
        for op_name, params in op_params.items():
            control = ControlParams()
            control.params = dict(zip(params['joint_names'], params['params']))
            #print control.params
            op = OperationPoint(op_name, np.array(params['state']), control)
            self.gain_scheduling.add_operation_point(op)

        # Checkpoint manager params
        # chp_params = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/checkpoint_list')
        # for chp_name,arm_states in chp_params.items():
        #   chp = OperationPoint(chp_name, np.array(arm_states['state']))
        #   self.checkpoint.add_operation_point(chp)

        return JointTrajectoryActionController.initialize(self)
      
    def start(self):
        # Publisher fro joint states
        self.joint_states_pub = rospy.Publisher(self.joint_states_topic, SensorJointState, queue_size = 20)
        Thread(target=self.gain_scheduling_loop).start()
        #Thread(target=self.checkpoint_loop).start()
        JointTrajectoryActionController.start(self)

    def process_trajectory(self, traj):
        # Update motor gains using last position in trajectory
        self.update_gains(traj.points[-1].positions)
        JointTrajectoryActionController.process_trajectory(self, traj)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Header
            self.msg.header.stamp = rospy.Time.now()
            self.sensor_joint_state.header.stamp = rospy.Time.now()
            
            # Publish current joint state
            for i, joint in enumerate(self.joint_names):
                state = self.joint_states[joint]
                self.msg.actual.positions[i] = state.current_pos
                self.msg.actual.velocities[i] = abs(state.velocity)
                self.msg.error.positions[i] = self.msg.actual.positions[i] - self.msg.desired.positions[i]
                self.msg.error.velocities[i] = self.msg.actual.velocities[i] - self.msg.desired.velocities[i]
                # Sensor joint state
                self.sensor_joint_state.position[i] = state.current_pos
                self.sensor_joint_state.velocity[i] = state.velocity
                self.sensor_joint_state.effort[i] = state.load
            # Publish msgs
            self.joint_states_pub.publish(self.sensor_joint_state)
            self.state_pub.publish(self.msg)
            rate.sleep()

    def gain_scheduling_loop(self):
        rate = rospy.Rate(1.0)
        rospy.sleep(1.0) # sleep for get a valid joint state msg
        # set initial operation point
        self.gain_scheduling.init(self.msg.actual.positions) # TODO Data race!
        while self.running and not rospy.is_shutdown():
            # check update
            if self.gain_scheduling.update(self.msg.actual.positions): # TODO Data race!
                control_params = self.gain_scheduling.get_params()
                for motor_name, motor_params in control_params.params.items():
                    # update motor params
                    rospy.loginfo('Setting {} with params {}'.format(motor_name, motor_params))
                    self.joint_to_controller[motor_name].update_control_params(motor_params)

            rate.sleep()

    def update_gains(self, positions):
        if self.gain_scheduling.update(positions):
            control_params = self.gain_scheduling.get_params()
            for motor_name, motor_params in control_params.params.items():
                # update motor params
                rospy.loginfo('Setting {} with params {}'.format(motor_name, motor_params))
                self.joint_to_controller[motor_name].update_control_params(motor_params)

    # def checkpoint_loop(self):
        # rate = rospy.Rate(1.0)
        # rospy.sleep(2.0) # sleep for get a valid joint state msg
        # # set initial operation point
        # self.checkpoint.init(self.msg.actual.positions) # TODO Data race!
        # while self.running and not rospy.is_shutdown():
        #   # check update
        #   self.checkpoint.update(self.msg.actual.positions)
        #   rate.sleep()

class GainScheduling():
    """
    Gain Scheduling
    @param threshold: Joint distance (norm) threshold for operation point change
    """
    def __init__(self, threshold = 1.0):
        self.operation_point = dict()
        self.threshold = threshold
        self.current_op = None

    def add_operation_point(self, op):
        self.operation_point[op.name] = op

    def update(self, actual_positions):
        # get current state
        current_state = np.array(actual_positions)
        for name, op in self.operation_point.items():
            # continue for current op
            if name == self.current_op.name:
                continue

            # change operation point
            dist = np.linalg.norm(current_state - op.state)
            #rospy.loginfo('Distance to {0}={1:.2f}'.format(op.name,dist))
            if dist < self.threshold:
                # update current position
                rospy.loginfo('Change operation point to {0}'.format(op.name))
                self.current_op = op
                return True
            # op dont change
        return False

    def get_params(self):
        return self.current_op.params
          

    """
    Update current operation point, use only on start
    @param actual_positions: List of actual joint positions
    """
    def init(self, actual_positions):
        # get current state
        current_state = np.array(actual_positions)
        current_op = None
        d = float('inf')
        # get nearest state
        for name, op in self.operation_point.items():
            test_d = np.linalg.norm(current_state - op.state)
            if test_d < d:
                d = test_d
                current_op = op
        rospy.loginfo('Init operation point in {0}'.format(current_op.name))
        self.current_op = current_op

# class Checkpoint():
#   """
#   Checkpoint
#   @param threshold: Joint distance (norm) threshold for operation point change
#   """
#   def __init__(self, threshold = 1.0, controller_namespace = ''):
#     self.states = dict()
#     self.threshold = threshold
#     self.current_op = None
#     self.state_publisher = rospy.Publisher(controller_namespace+'/joint_trajectory/current_state', String, queue_size=10)

#   def add_operation_point(self, chp):
#     self.state[chp.name] = chp

#   def update(self, actual_positions):
#     # get current state
#     current_state = np.array(actual_positions)
#     for name, op in self.states.items():
#       # continue for current op
#       if name == self.current_op.name:
#         continue

#       # change operation point
#       dist = np.linalg.norm(current_state - op.state)
#       #rospy.loginfo('Distance to {0}={1:.2f}'.format(op.name,dist))
#       if dist < self.threshold:
#         # update current position
#         rospy.loginfo('Change operation point to {0}'.format(op.name))
#         self.current_op = op
#         return True
#     # op dont change
#     return False

#   def get_params(self):
#     return self.current_op.params

#   """
#   Update current operation point, use only on start
#   @param actual_positions: List of actual joint positions
#   """
#   def init(self, actual_positions):
#     # get current state
#     current_state = np.array(actual_positions)
#     current_op = None
#     d = float('inf')
#     # get nearest state
#     for name, op in self.states.items():
#       test_d = np.linalg.norm(current_state - op.state)
#       if test_d < d:
#         d = test_d
#         current_op = op
#     rospy.loginfo('Init operation point in {0}'.format(current_op.name))
#     self.current_op = current_op


class OperationPoint():
    def __init__(self, name, state, ctrl_params):
        self.name = name
        self.state = state # numpy array
        self.params = ctrl_params

class ControlParams():
    def __init__(self):
        self.params = dict()

    # motor_params [p,i,d] or [slope, margin, punch]
    def add_control_param(self, motor_name, motor_params):
        self.params[motor_name] = motor_params
    
