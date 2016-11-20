#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Joint space control using joint trajectory action
"""
__author__ = "Rodrigo Muñoz"

import copy
from threading import Lock
import numpy as np
# ROS Core
import rospy
import actionlib
# ROS Messages
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
# Robot skill
from bender_core.robot_skill import RobotSkill

class LightHead(RobotSkill):
    """
    Joint space control using joint trajectory action for light head
    """
    _type = "head"

    # Class constants
    JOINT_NAMES = ["light_head_yaw_joint", "light_head_pitch_joint"]
    """list of str: Joints names"""

    NUM_JOINTS = 2
    """int: Number of joints"""

    def __init__(self):
        """
        Joint space control using joint trajectory action for light head
        """
        super(LightHead, self).__init__()
        self._description = "Joint space control for light head"
        # Head topic
        self._jta_topic = "/bender/light_head_controller/follow_joint_trajectory"
        # Joint names
        self.joint_names = LightHead.JOINT_NAMES
        # Empty joint state message
        self._joint_state = JointState()
        self._joint_state.name = self.joint_names
        self._joint_state.position = [0.0]*LightHead.NUM_JOINTS
        self._joint_state.velocity = [0.0]*LightHead.NUM_JOINTS
        self._joint_state.effort = [0.0]*LightHead.NUM_JOINTS
        # ROS clients (avoid linter warnings)
        self._jta_client = None

    def _update_joint_state(self, msg):
        """
        Update joint positions.
        """
        pass

    def get_joint_state(self):
        """
        Get current joint state.

        Returns:
            sensor_msgs.msg.JointState: Joint state.
        """
        # Acquire lock and return a complete copy
        with self._joint_state_lock:
            return copy.deepcopy(self._joint_state)

    def get_joint_names(self):
        """
        Get joint names.

        Returns:
            :obj:`list` of :obj:`str`: Joint names in order.
        """
        return copy.deepcopy(self.joint_names)

    # TODO(mpavez) Add timeout param?
    def check(self, timeout=1.0):
        # Check client for joint trajectory action (JTA)
        jta_client = actionlib.SimpleActionClient(self._jta_topic, FollowJointTrajectoryAction)
        # Wait for the JTA server to start or exit
        if not jta_client.wait_for_server(timeout=rospy.Duration(timeout)):
            self.logerr("Joint trajectory action server for \"{0}\" not found".format(self.name))
            return False
        self.log.debug("Joint trajectory action server for \"{0}\" [OK]".format(self.name))
        # Check joint_states topic
        # try:
        #     msg = rospy.wait_for_message(self._joint_state_topic, JointState, timeout=timeout)
        # except rospy.ROSException:
        #     self.logerr("Topic \"{0}\" not already published".format(self._joint_state_topic))
        #     return False
        # Check arm joints in message
        # for joint in self.joint_names:
        #     if not joint in msg.name:
        #         self.logerr("Topic \"{0}\" does not contain \"{1}\" joints".format(
        #             self._joint_state_topic, self.name))
        #         return False
        # self.logdebug("Topic \"{0}\" published [OK]".format(self._joint_state_topic))
        # return True

    def setup(self):
        # Joint trajectory action (JTA)
        self._jta_client = actionlib.SimpleActionClient(self._jta_topic, FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        return True

    def shutdown(self):
        self.logwarn("Shutdown \"{0}\" skill, calling cancel goals...".format(self.name))
        # Cancel goals
        self._jta_client.cancel_all_goals()
        return True

    def start(self):
        self.logdebug("Start \"{0}\" skill".format(self.name))
        return True

    def pause(self):
        self.logdebug("Pause \"{0}\" skill".format(self.name))
        return True

    # Head movement related methods
    def stop(self):
        """
        Cancel all goals.
        """
        self.logwarn("Stop \"{0}\", calling cancel goals...".format(self.name))
        self._jta_client.cancel_all_goals()

    def send_joint_goal(self, roll=0.0, pitch=0.0, interval=3.0, segments=10):
        """
        Send joint goal reference to the head.

        This function use linear interpolation between current position (obtained via joint_states topic)
        and joint goal.

        Returns:
            joint_goal (list of float): Joint target configuration, must follow arm.get_joint_names() order.
            interval (float): Time interval between current position and joint goal.

        Examples:
            >>> arm.send_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Send to home position
        """
        # Check joint state time stamp
        rospy.sleep(0.05)
        current_state = self.get_joint_state()
        if (rospy.Time.now() - current_state.header.stamp) > rospy.Duration(1.0):
            self.logerr("Current position has not been updated, check \"{}\" topic.".format(self._joint_state_topic))
            return
        # Create new goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.get_joint_names()
        dt = interval/segments
        t = 0.1
        inter_points = list()
        for i in range(LightHead.NUM_JOINTS):
            # TODO(rorromr) Use parabolic interpolation
            inter_points.append(np.linspace(current_state.position[i], joint_goal[i], segments))
        for j in range(segments):
            point = JointTrajectoryPoint()
            point.positions = [joint[j] for joint in inter_points]
            t += dt
            point.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(point)
        # Send goal to JTA
        self.loginfo('Sending new goal for \"{0}\"'.format(self.name))
        self._jta_client.send_goal(goal)

    def wait_for_motion_done(self, timeout=0.0):
        """
        Blocks until gripper motion is done

        Args:
            timeout (float): Max time to block before returning. A zero timeout is interpreted as an infinite timeout.

        Returns:
            bool: True if the goal finished. False if the goal didn't finish within the allocated timeout.
        """
        self.log.info('Waiting for \"{0}\" motion'.format(self.name))
        return self._jta_client.wait_for_result(rospy.Duration(timeout))

    def get_result(self):
        """
        Get movement result

        Returns:
            control_msgs.msg.FollowJointTrajectoryResult: If the goal finished.
            None: If the goal didn't finish.
        """
        return self._jta_client.get_result()
        
    def look_at(self, pose):
        pass
