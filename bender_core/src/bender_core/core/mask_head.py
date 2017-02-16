#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Joint space control using joint trajectory action
"""
__author__ = "Cristopher Gomez, Rodrigo Munoz"

import copy
from threading import Lock
import numpy as np
import math
# ROS Core
import rospy
import actionlib
import tf
# ROS Messages
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
# Robot skill
from bender_core.robot_skill import RobotSkill

class MaskHeadSkill(RobotSkill):
    """
    Joint space control using joint trajectory action for mask head
    """
    _type = "mask_head"

    # Class constants
    JOINT_NAMES = ["head_yaw_joint", "head_pitch_joint"]
    """list of str: Joints names"""

    NUM_JOINTS = 2
    """int: Number of joints"""

    YAW_HOME_POSITION = 0.0
    """float: Roll angle home postion"""

    PITCH_HOME_POSITION = 0.0
    """float: Pitch angle home postion"""

    PITCH_MIN_POSITION = 0.6
    """float: Minimum pitch angle"""

    REF_FRAME = "bender/head_link"
    """str: Reference frame for transformations"""

    def __init__(self):
        """
        Joint space control using joint trajectory action for mask head
        """
        super(MaskHeadSkill, self).__init__()
        self._description = "Joint space control for mask head"
        # Head topic
        self._jta_topic = "/bender/head_controlle/follow_joint_trajectory"
        # Name
        self.name = MaskHeadSkill._type
        # Joint names
        self.joint_names = MaskHeadSkill.JOINT_NAMES
        # Empty joint state message
        self._joint_state = JointState()
        self._joint_state.name = self.joint_names
        self._joint_state.position = [0.0]*MaskHeadSkill.NUM_JOINTS
        self._joint_state.velocity = [0.0]*MaskHeadSkill.NUM_JOINTS
        self._joint_state.effort = [0.0]*MaskHeadSkill.NUM_JOINTS
        # ROS clients (avoid linter warnings)
        self._jta_client = None
        # Only for check, joint states is obtained using robot context
        self._joint_state_topic = self.context.get_joint_state_topic()
        
    def get_joint_state(self):
        """
        Get current joint state.

        Returns:
            sensor_msgs.msg.JointState: Joint state.
        """
        robot_joint_state = self.context.get_joint_state()
        self._joint_state.header = robot_joint_state.header
        for i,joint in enumerate(self.joint_names):
            try:
                
                joint_idx = robot_joint_state.name.index(joint)
                self._joint_state.position[i] = robot_joint_state.position[joint_idx]
                self._joint_state.velocity[i] = robot_joint_state.velocity[joint_idx]
                self._joint_state.effort[i] = robot_joint_state.effort[joint_idx]
            except ValueError:
                continue
        return copy.deepcopy(self._joint_state)

    def get_joint_names(self):
        """
        Get joint names.

        Returns:
            :obj:`list` of :obj:`str`: Joint names in order.
        """
        return copy.deepcopy(self.joint_names)

    def check(self, timeout=1.0):
        # Check client for joint trajectory action (JTA)
        jta_client = actionlib.SimpleActionClient(self._jta_topic, FollowJointTrajectoryAction)
        # Wait for the JTA server to start or exit
        if not jta_client.wait_for_server(timeout=rospy.Duration(timeout)):
            self.logerr("Joint trajectory action server for \"{0}\" not found".format(self.name))
            return False
        self.log.debug("Joint trajectory action server for \"{0}\" [OK]".format(self.name))
        # Check joint_states topic
        try:
            msg = rospy.wait_for_message(self._joint_state_topic, JointState, timeout=timeout)
        except rospy.ROSException:
            self.logerr("Topic \"{0}\" not already published".format(self._joint_state_topic))
            return False
        # Check head joints in message
        for joint in self.joint_names:
            if not joint in msg.name:
                self.logerr("Topic \"{0}\" does not contain \"{1}\" joints".format(
                    self._joint_state_topic, self.name))
                return False
        self.logdebug("Topic \"{0}\" published [OK]".format(self._joint_state_topic))
        return True

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
        self.stop()
        return True

    # Head movement related methods
    def stop(self):
        """
        Cancel all goals.
        """
        self.logwarn("Stop \"{0}\", calling cancel goals...".format(self.name))
        self._jta_client.cancel_all_goals()

    def send_joint_goal(self, yaw, pitch, interval=1.0, segments=5):
        """
        Send joint goal reference to the head.

        This function use linear interpolation between current position (obtained via joint_states topic)
        and joint goal.

        Args:
            yaw (float): Roll angle target.
            pitch (float): Pitch angle target.
            interval (float): Time interval between current position and joint goal.

        Examples:
            >>> arm.send_joint_goal(yaw=0.0, pitch=0.0) # Send to home position
        """
        # Check joint state time stamp
        rospy.sleep(0.05)
        joint_goal = [yaw, pitch]
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
        for i in range(MaskHeadSkill.NUM_JOINTS):
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

    def home(self):
        """
        Move head to home position.
        """
        self.send_joint_goal(yaw=MaskHeadSkill.YAW_HOME_POSITION, pitch=MaskHeadSkill.PITCH_HOME_POSITION)

    def look_at_ground(self):
        """
        Look at the ground.
        """
        self.send_joint_goal(yaw=MaskHeadSkill.YAW_HOME_POSITION, pitch=0.9*MaskHeadSkill.PITCH_MIN_POSITION)

    def look_at(self, pose):
        """
        Look at pose.

        Args:
            pose (geometry_msgs.msg.PoseStamped): Target pose stamped to look, frame transform is managed internally,
                so 'frame_id' can be any frame.
        """
        # Transform to reference frame
        pose.header.stamp = rospy.Time() # Use last transform
        try:
            target_pose = self.context.get_tf_listener().transformPose(MaskHeadSkill.REF_FRAME, pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.logerr("Error on transform from \"{}\" to \"{}\"".format(pose.header.frame_id, MaskHeadSkill.REF_FRAME))
            return
        # Trasnform based on spherical coordinate
        x,y,z = target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z
        yaw = math.atan2(y,x)
        pitch = -(math.pi/2 - math.acos(z/math.sqrt(x*x + y*y + z*z)))
        self.send_joint_goal(yaw, pitch)
