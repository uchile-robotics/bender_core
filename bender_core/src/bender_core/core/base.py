#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math

from bender_core.robot_skill import RobotSkill
from bender_core.core.joy import JoySkill
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class BaseSkill(RobotSkill):
    """
    The BaseSkill

    Depends on: Joystick
    """
    _type = "base"

    def __init__(self):
        super(BaseSkill, self).__init__()
        self._description = "the base skill"
        self.register_dependency(JoySkill.get_type())
        self.pose_pub = rospy.Publisher("/bender/nav/cmd_vel", Twist, queue_size=1)
        self.linear_vel = 0.3
        self.angular_vel = 0.3
        self.curr_pose = Odometry()

    def _update_pos(self, data):
        self.curr_pose = data

    def _distance(self, pos1, pos2):
        dist = math.sqrt((pos1.pose.pose.position.x - pos2.pose.pose.position.x) ** 2 + (pos1.pose.pose.position.y - pos2.pose.pose.position.y) ** 2 + (pos1.pose.pose.position.z - pos2.pose.pose.position.z) ** 2)
        return dist

    def _rotation(self, pos1, pos2):
        ang1 = math.asin(pos1.pose.pose.orientation.z) * 2
        ang2 = math.asin(pos2.pose.pose.orientation.z) * 2
        return abs((ang1 - ang2)) * 180 / math.pi

        
    def check(self):
        rospy.loginfo("{skill: %s}: check()." % self._type)
        return True

    
    def setup(self):
        rospy.loginfo("{skill: %s}: setup()." % self._type)
        return True


    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return True
        

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True


    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)
        return True

    def move(self, distance=0.0):
        """
        This method moves the base by "distance" meters.

        Primitive movement of a determined distance, a given distance of 0 will result in continuous movement.
        A negative distance will result in backwards movement, a positive distance in forward movement.

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        rospy.loginfo("{skill: %s}: move_forward(). Moving by %f meters." % (self._type, abs(distance)))

        covered = 0
        rate = rospy.Rate(10)
        pose_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self._update_pos)
        ini_pose = self.curr_pose
        signo = 1 if distance > 0 else -1
        try:
            while not rospy.is_shutdown():
                covered = self._distance(self.curr_pose, ini_pose)
                rospy.loginfo("Distance covered: %f meters" % (covered))
                if distance != 0:
                    if covered >= abs(distance):
                        vel = Twist()
                        
                        self.pose_pub.publish(vel)
                        rospy.loginfo("Goal reached!")
                        return True
                    else:
                        vel = Twist()
                        vel.linear.x = signo * self.linear_vel
                        
                        self.pose_pub.publish(vel)
                rate.sleep()
        except rospy.ROSException:
            rospy.loginfo("Couldn't reach goal :(")
            return False

    def move_forward(self, distance=0.0):
        """
        This method moves the base forward by "distance" meters.

        Primitive movement of a determined distance, a given distance of 0 will result in continuous movement

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        rospy.loginfo("{skill: %s}: move_forward(). Moving forward by %f meters." % (self._type, distance))

        return self.move(distance)

    def move_backwards(self, distance=0.0):
        """
        This method moves the base backwards by "distance" meters.

        Primitive movement of a determined distance, a given distance of 0 will result in continuous movement

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        return self.move(-distance)

    def rotate(self, angle=0.0):
        """
        This method rotates the base by "angle" degrees.

        Primitive rotation of a determined angle, a given angle of 0 will result in continuous movement.
        A negative angle will result in clockwise rotation, a positive angle in counter-clockwise rotation.

        Args:
            angle (float): degrees to rotate the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        rospy.loginfo("{skill: %s}: rotate_left(). Rotating %f degrees counter-clockwise." % (self._type, angle))
        covered = 0.0
        rate = rospy.Rate(10)
        pose_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self._update_pos)
        ini_pose = self.curr_pose
        signo = 1 if angle > 0 else -1
        try:
            while not rospy.is_shutdown():
                covered = self._rotation(self.curr_pose, ini_pose)
                rospy.loginfo("Rotated angle: %f degrees" % (covered))
                if angle != 0:
                    if covered >= abs(angle):
                        vel = Twist()

                        self.pose_pub.publish(vel)
                        rospy.loginfo("Goal reached!")
                        return True
                    else:
                        vel = Twist()
                        vel.angular.z = signo * self.angular_vel

                        self.pose_pub.publish(vel)
                rate.sleep()
        except rospy.ROSException:
            rospy.loginfo("Couldn't reach goal :(")
            return False

    def rotate_right(self, angle = 0.0):
        """
        This method rotates the base clockwise by "angle" degrees.

        Primitive rotation of a determined angle, a given angle of 0 will result in continuous movement.

        Args:
            angle (float): degrees to rotate the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        return self.rotate(-angle)

    def rotate_left(self, angle = 0.0):
        """
        This method rotates the base counter-clockwise by "angle" degrees.

        Primitive rotation of a determined angle, a given angle of 0 will result in continuous movement.

        Args:
            angle (float): degrees to rotate the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        return self.rotate(angle)

    def _move_forward_private_version(self, distance=0.0):
        """
        Methods starting with '_' will not be displayed in the skill overview
        """
        return True
