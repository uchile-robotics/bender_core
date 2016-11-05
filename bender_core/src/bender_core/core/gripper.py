#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gripper control using gripper command action
"""
__author__ = "Rodrigo Muñoz"

# ROS Core
import rospy
import actionlib
# ROS Messages
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# Robot skill
from bender_core.robot_skill import RobotSkill

class GripperSkill(RobotSkill):
    """
    Base class for gripper control using gripper command action.
    """

    _type = "gripper"

    L_GRIPPER = "l_gripper"
    """str: Left gripper name"""

    R_GRIPPER = "r_gripper"
    """str: Right gripper name"""

    CLOSE_POSITION = 0.0
    """float: Gripper close postion"""

    OPEN_POSITION = 0.6
    """float: Gripper open postion"""

    def __init__(self, gripper_name):
        """
        Base class for gripper control using gripper command action

        Args:
            gripper_name (str): Gripper name (must be "l_gripper" or "r_gripper").
        Raises:
            TypeError: If `gripper_name` is not a string.
            ValueError: If `gripper_name` is not "l_gripper" or "r_gripper".
        """
        super(GripperSkill, self).__init__()
        self._description = "Gripper control using gripper command action"
        # Check gripper name
        if not isinstance(gripper_name, str):
            raise TypeError("gripper_name must be a string")
        if not (gripper_name == GripperSkill.L_GRIPPER or gripper_name == GripperSkill.R_GRIPPER):
            raise ValueError("gripper_name must be \"l_gripper\" or \"r_gripper\"")
        # Get arm name and side
        self.name = gripper_name
        self.side = gripper_name[0]
        # Gripper command action topic
        self._gca_topic = "/bender/{0}_controller/gripper_action".format(self.name)
        # ROS clients (avoid linter warnings)
        self._gca_client = None

    def check(self, timeout=1.0):
        # Check client for gripper command action (GCA)
        gca_client = actionlib.SimpleActionClient(self._gca_topic, GripperCommandAction)
        # Wait for the GCA server to start or exit
        if not gca_client.wait_for_server(timeout=rospy.Duration(timeout)):
            self.logerr("Gripper command action server for \"{0}\" not found".format(self.name))
            return False
        self.logdebug("Gripper command action server for \"{0}\" [OK]".format(self.name))
        return True

    def setup(self):
        # Gripper command action (GCA)
        self._gca_client = actionlib.SimpleActionClient(self._gca_topic, GripperCommandAction)
        rospy.sleep(0.1)
        return True

    def shutdown(self):
        self.logwarn("Shutdown \"{0}\" skill, calling cancel goals...".format(self.name))
        # Cancel goals
        self._gca_client.cancel_all_goals()
        return True

    def start(self):
        self.logdebug("Start \"{0}\" skill".format(self.name))
        return True

    def pause(self):
        self.logdebug("Pause \"{0}\" skill".format(self.name))
        return True

    # Gripper related mothods
    def send_goal(self, position, effort=0.5):
        """
        Send gripper command.

        Args:
            position (float): Joint target configuration.
            effort (float): Max effort used in the movement. Must be between 0.0 and 1.0. Value zero may cause null movement.

        Examples:
            >>> gripper.send_goal(0.3, 0.8)
        """
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self.loginfo('Sending new goal for \"{0}\" with position={1:.2f} effort={2:.2f}'.format(self.name, position, effort))
        self._gca_client.send_goal(goal)

    def close(self, effort=0.5):
        """
        Close the gripper to default close position and wait for movement done.

        Args:
            effort (float): Max effort used in close movement. Must be between 0.0 and 1.0. Value zero may cause null movement.

        Returns:
            bool: True if the goal finished, False otherwise.
        """
        self.loginfo('Closing {0}.'.format(self.name))
        self.send_goal(GripperSkill.CLOSE_POSITION, effort)
        return self.wait_for_motion_done()

    def open(self, effort=0.8):
        """
        Open the gripper to default open position and wait for movement done.

        Args:
            effort (float): Max effort used in close movement. Must be between 0.0 and 1.0. Value zero may cause null movement.

        Returns:
            bool: True if the goal finished, False otherwise.
        """
        self.loginfo('Opening {0}.'.format(self.name))
        self.send_goal(GripperSkill.OPEN_POSITION, effort)
        return self.wait_for_motion_done()

    def wait_for_motion_done(self, timeout=0.0):
        """
        Blocks until gripper motion is done

        Args:
            timeout (float): Max time to block before returning. A zero timeout is interpreted as an infinite timeout.

        Returns:
            bool: True if the goal finished. False if the goal didn't finish within the allocated timeout.
        """
        self.loginfo('Waiting for \"{0}\" motion'.format(self.name))
        return self._gca_client.wait_for_result(rospy.Duration(timeout))

    def get_result(self):
        """
        Get movement result

        Returns:
            control_msgs.msg.GripperCommandResult: If the goal finished.
            None: If the goal didn't finish.
        """
        return self._gca_client.get_result()

class LeftGripperSkill(GripperSkill):
    """Left gripper control using gripper command action"""
    _type = "l_gripper"
    def __init__(self):
        super(LeftGripperSkill, self).__init__(GripperSkill.L_GRIPPER)

class RightGripperSkill(GripperSkill):
    """Right gripper control using gripper command action"""
    _type = "r_gripper"
    def __init__(self):
        super(RightGripperSkill, self).__init__(GripperSkill.R_GRIPPER)
