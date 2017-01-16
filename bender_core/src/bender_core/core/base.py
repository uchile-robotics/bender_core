#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math

from bender_core.robot_skill import RobotSkill
from bender_core.core.joy import JoySkill
from control_util.pid import PID
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

        self.linear_acc = 0.5
        self.max_linear_vel = 0.5
        self.min_linear_vel = 0.02
        self.mean_lin_vel = 0.01

        self.angular_acc = 0.3
        self.max_angular_vel = 0.5
        self.min_angular_vel = 0.04
        self.mean_ang_vel = 0.01

        self.prev_vel = 0
        self.max_linear_acc = 0.1
        self.max_angular_acc = 0.1

        self.curr_pose = Odometry()
        self.pub_rate = 10.0

        self.lin_pid = PID(kp = 1, kd = .01)
        self.ang_pid = PID(kp = 1, kd = 0.01)

    def _update_pos(self, data):
        self.curr_pose = data

    def _distance(self, pos1, pos2):
        """
        This method calculates the distance between two Odometry type positions.

        Args:
            pos1 (Odometry):
                first Odometry type message position
            pos2 (Odometry):
                second Odometry type message position

        Returns:
            float: Distance between the two Odometry positions
        """
        dist = math.sqrt((pos1.pose.pose.position.x - pos2.pose.pose.position.x) ** 2 + (pos1.pose.pose.position.y - pos2.pose.pose.position.y) ** 2 + (pos1.pose.pose.position.z - pos2.pose.pose.position.z) ** 2)
        return dist

    def _rotation_rad(self, pos1, pos2, angle):
        """
        This method calculates the distance between two Odometry type orientations.

        Args:
            pos1 (Odometry):
                first Odometry type message
            pos2 (Odometry):
                second Odometry type message

        Returns:
            float: Radians between the two Odometry orientations
        """
        ang1 = math.acos(pos1.pose.pose.orientation.z)
        ang2 = math.acos(pos2.pose.pose.orientation.z)
        diff = abs(ang1 - ang2) * 2
        return diff if diff < angle else (2 * math.pi - diff)

    def _rotation(self, pos1, pos2):
        """
        This method calculates the distance between two Odometry type orientations.

        Args:
            pos1 (Odometry):
                first Odometry type message
            pos2 (Odometry):
                second Odometry type message

        Returns:
            float: Sexagecimal degrees between the two Odometry orientations
        """
        return self._rotation_rad(pos1, pos2) * 180 / math.pi

    def _get_linear_vel(self, distance, covered):
        """
        This method calculates the linear velocity to be sent based on distance to cover.

        Args:
            distance (float): Distance to move the base
            covered (float): Distance covered so far

        Result:
            float: Linear velocity to be published
        """
        speed = self.lin_pid.compute_output(abs(distance) - covered + abs(distance) / 100.0)
        speed2 = (speed if abs(speed) < self.max_linear_vel else abs(speed) / speed * self.max_linear_vel)

        return speed2 if abs(speed2) > self.min_linear_vel else abs(speed2) / speed2 * self.min_linear_vel

    def _get_angular_vel(self, angle, covered):
        """
        This method calculates the angular velocity to be sent based on distance to cover.

        Args:
            angle (float): Angle to move the base
            covered (float): Distance covered so far

        Result:
            float: Angular velocity to be published
        """
        speed = self.ang_pid.compute_output(angle - covered)
        speed2 = (speed if abs(speed) < self.max_angular_vel else abs(speed) / speed * self.max_angular_vel)

        return speed2 if abs(speed2) > self.min_angular_vel else abs(speed2) / speed2 * self.min_angular_vel
        
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

    def move(self, distance=0.0, timeout=None):
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
        rospy.loginfo("{skill: %s}: move(). Moving by %f meters." % (self._type, abs(distance)))

        covered = 0
        rate = rospy.Rate(self.pub_rate)
        pose_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self._update_pos)
        ini_pose = self.curr_pose
        signo = 1 if distance > 0 else -1
        self.lin_pid.initialize()

        start = rospy.get_rostime()
        timeout = rospy.Duration(1.0 * timeout) if timeout is not None else rospy.Duration(abs(1.0 * distance) / self.mean_lin_vel)
        try:
            self.curr_lin_vel = 0
            while not rospy.is_shutdown():

                elapsed_time = rospy.get_rostime() - start
                covered = self._distance(self.curr_pose, ini_pose)
                rospy.loginfo("Distance covered: %f meters" % (covered))
                if elapsed_time < timeout:
                    if distance != 0:
                        if abs(distance) - covered <= abs(distance) / 100.0:
                            vel = Twist()
                            
                            self.pose_pub.publish(vel)
                            rospy.loginfo("Goal reached! Final position: [x: %f, y: %f, z: %f]" % (self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y, self.curr_pose.pose.pose.position.z))
                            rospy.loginfo("Error: %f m or %f percent" % (abs(covered - abs(distance)), abs((covered - abs(distance)) / distance) * 100))
                            return True
                        else:
                            vel = Twist()
                            vel.linear.x = signo * self._get_linear_vel(abs(distance), covered)
                            
                            rospy.loginfo("Moving at %f m/s" % (vel.linear.x))
                            
                            self.pose_pub.publish(vel)
                    else:
                        vel = Twist()
                        vel.linear.x = signo * self._get_linear_vel(abs(distance), covered)
                        
                        rospy.loginfo("{Continuous movement} Moving at %f m/s" % (vel.linear.x))
                        
                        self.pose_pub.publish(vel)
                else:
                    vel = Twist()
                            
                    self.pose_pub.publish(vel)
                    rospy.loginfo("Timeout of %f seconds reached, ending movement." % (timeout.to_sec()))
                    rospy.loginfo("Error: %f m or %f percent" % (abs(covered - abs(distance)), abs((covered - abs(distance)) / distance) * 100))
                    return False
                rate.sleep()
        except rospy.ROSException:
            rospy.loginfo("Couldn't reach goal :(")
            return False

    def move_forward(self, distance=0.0, timeout=None):
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

        return self.move(distance, timeout)

    def move_backwards(self, distance=0.0, timeout=None):
        """
        This method moves the base backwards by "distance" meters.

        Primitive movement of a determined distance, a given distance of 0 will result in continuous movement

        Args:
            distance (float): Distance to move the base.
                Defaults to 0.0

        Returns:
            bool: True on success, False otherwise 
        """
        return self.move(-distance, timeout)

    def rotate_rad(self, angle = 0.0, signo = 1, timeout = None):
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
        rospy.loginfo("{skill: %s}: rotate_rad(). Rotating %f rads." % (self._type, angle))
        covered = 0.0
        rate = rospy.Rate(10)
        pose_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self._update_pos)
        ini_pose = self.curr_pose
        self.ang_pid.initialize()
        rospy.loginfo("Starting at: %f rads" % (math.acos(ini_pose.pose.pose.orientation.w) * 2))

        start = rospy.get_rostime()
        timeout = rospy.Duration(1.0 * timeout) if timeout is not None else rospy.Duration(abs(1.0 * angle) / self.mean_ang_vel)
        try:
            while not rospy.is_shutdown():
                elapsed_time = rospy.get_rostime() - start
                covered = self._rotation_rad(self.curr_pose, ini_pose, angle)
                rospy.loginfo("Rotated angle: %f rads, Current pos: %f" % (covered, math.acos(self.curr_pose.pose.pose.orientation.w) * 2))
                if elapsed_time < timeout:
                    if angle != 0:
                        if angle - covered <= angle / 100.0:
                            vel = Twist()

                            self.pose_pub.publish(vel)
                            rospy.loginfo("Goal reached!")
                            rospy.loginfo("Error: %f rads or %f percent" % (shortest_angular_distance(covered, angle), abs(shortest_angular_distance(covered, angle) / angle) * 100))
                            return True
                        else:
                            vel = Twist()
                            vel.angular.z = signo * self._get_angular_vel(angle, covered)

                            rospy.loginfo("Moving at %f rad/s" % (vel.angular.z))
                            self.pose_pub.publish(vel)
                    else:
                        vel = Twist()
                        vel.angular.z = signo * self._get_angular_vel(angle, covered)
                        
                        rospy.loginfo("{Continuous movement} Moving at %f rad/s (Not currently working)" % (vel.linear.x))
                        
                        self.pose_pub.publish(vel)
                else:
                    vel = Twist()
                            
                    self.pose_pub.publish(vel)
                    rospy.loginfo("Timeout of %f seconds reached, ending movement." % (timeout.to_sec()))
                    rospy.loginfo("Error: %f rads or %f percent" % (shortest_angular_distance(covered, angle), abs(shortest_angular_distance(covered, angle) / angle) * 100))
                    return False

                rate.sleep()
        except rospy.ROSException:
            rospy.loginfo("Couldn't reach goal :(")
            return False

    def rotate(self, angle=0.0, signo=1, timeout = None):
        return self.rotate_rad(angle * math.pi / 180.0, signo)

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
        return self.rotate(angle, -1)

    def rotate_left(self, angle = 0.0, timeout = None):
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



### Angle functions ###

def normalize_angle_positive(angle):
    """ Normalizes the angle to be 0 to 2*pi
        It takes and returns radians. """
    return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)

def normalize_angle(angle):
    """ Normalizes the angle to be -pi to +pi
        It takes and returns radians."""
    a = normalize_angle_positive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a

def shortest_angular_distance(from_angle, to_angle):
    """ Given 2 angles, this returns the shortest angular
        difference.  The inputs and ouputs are of course radians.
 
        The result would always be -pi <= result <= pi. Adding the result
        to "from" will always get you an equivelent angle to "to".
    """
    return normalize_angle(to_angle-from_angle)
