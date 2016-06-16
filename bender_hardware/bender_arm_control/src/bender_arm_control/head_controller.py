#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division


__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'


# Thread
from threading import Thread
# ROS
import rospy
from sensor_msgs.msg import JointState
from bender_arm_control.srv import HeadPosition, HeadPositionResponse, HeadPositionRequest 

class HeadController():
    def __init__(self, controller_namespace, controllers):
        self.controller_namespace = controller_namespace
        
        self.controller = controllers[0] # JointPositionController
        self.joint_name = self.controller.joint_name
        self.joint_state = self.controller.joint_state
        # joint_states base msg
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [self.joint_name]
        self.joint_state_msg.position = [0.0]
        self.joint_state_msg.velocity = [0.0]
        self.joint_state_msg.effort = [0.0]

    def initialize(self):
        # Topic name for publish joint_states
        self.joint_states_topic = rospy.get_param(self.controller_namespace + '/head_controller_node/joint_states_topic', 'joint_states')
        # Parametro rate para /joint_states
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/head_controller_node/state_update_rate', 50)

        return True
        
    def start(self):
        self.running = True
        # Publisher for joint states
        self.joint_states_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size = 20)

        self.pos_service = rospy.Service('rgbd_position', HeadPosition, self.pos_callback)

        # Thread for joint states
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False

    def pos_callback(self, req):
        rospy.loginfo('Sending request for position={:.2f}'.format(req.position.data))
        self.controller.process_command(req.position)
        return HeadPositionResponse(reach=self.running) # @TODO return something...


    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Header
            self.joint_state_msg.header.stamp = rospy.Time.now()
            # Publish current joint state
            # Sensor joint state
            self.joint_state_msg.position[0] =  self.joint_state.current_pos
            self.joint_state_msg.velocity[0] =  self.joint_state.velocity
            self.joint_state_msg.effort[0] = self.joint_state.load
            # Publish msgs
            self.joint_states_pub.publish(self.joint_state_msg)
            rate.sleep()
