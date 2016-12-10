#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Robot context
"""
__author__ = "Rodrigo Muñoz"

import copy
from threading import Lock
# ROS Core
import rospy
import tf
# ROS Messages
from sensor_msgs.msg import JointState

class Context(object):
    """
    Robot context singleton
    """
    # Singleton instance
    __instance = None

    def __new__(cls, *a, **k):
        if Context.__instance is None:
            Context.__instance = ContextImpl(*a, **k)
        return Context.__instance

    def __getattr__(self, name):
        return getattr(Context.__instance, name)

    def __setattr__(self, name):
        return setattr(Context.__instance, name)

    @staticmethod
    def get_context():
        return Context()

class ContextImpl(object):
    """
    Robot context
    """
    
    JOINT_STATES_TOPIC="/bender/joint_states"

    def __init__(self, robot_name="bender"):
        """
        Robot context
        """
        # Ser robot name
        self.robot_name = robot_name
        # Arm topics
        self._joint_state_topic = ContextImpl.JOINT_STATES_TOPIC
        # Joint state subscriber
        self._joint_state_sub = rospy.Subscriber(self._joint_state_topic,
            JointState, self._update_joint_state)
        # Empty joint state message
        self._joint_state_lock = Lock()
        self._joint_state = JointState()
        # TF listener
        self.tf_listener = tf.TransformListener()

    def _update_joint_state(self, msg):
        """
        Update joint positions.
        """
        with self._joint_state_lock:
            self._joint_state = msg

    def get_joint_state(self):
        """
        Get current joint state.

        Returns:
            sensor_msgs.msg.JointState: Joint state.
        """
        # Acquire lock and return a complete copy
        with self._joint_state_lock:
            return copy.deepcopy(self._joint_state)

    def get_tf_listener(self):
        """
        Get a shared TF transform listener.

        Returns:
            tf.TransformListener: Transform listener.
        """
        return self.tf_listener

    def get_robot_name(self):
        """
        Get the robot name.

        Returns:
            str: Robot name.
        """
        return self.robot_name

    def get_joint_state_topic(self):
        """
        Get joint state topic.

        Returns:
            str: Joint state topic.
        """
        return self._joint_state_topic
