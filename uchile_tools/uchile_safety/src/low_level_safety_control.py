#!/usr/bin/env python
__author__ = 'Diego Bano'
__email__  = 'diego.bano@ug.uchile.cl'

import rospy
from math import sin, cos, atan2, pi, sqrt, pow as mpow
import numpy
import roslib
import tf
roslib.load_manifest("uchile_safety")

from threading import Thread, Lock
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from uchile_srvs.srv import Transformer


class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        # Variables for tf transformations
        self.laser_front_param = rospy.get_param("bender_sensors_laser_front_link",
                                                 "/bender/sensors/laser_front_link")
        self.laser_rear_param = rospy.get_param("bender_sensors_laser_rear_link",
                                                "/bender/sensors/laser_rear_link")
        self.bender_base_frame = rospy.get_param("base_link", "/bender/base_link")
        
        self.listener = tf.TransformListener()

        self.tf_client = rospy.ServiceProxy("/bender/tf/simple_pose_transformer/transform", Transformer)
        
        # Laser start up variables
        self.laser_front_closest_point = [float("inf"), pi]
        self.laser_rear_closest_point = [float("inf"), pi]

        ### Laser position transform ###
        # Front laser
        laser_pose = self.get_laser_front_base_transform(0, 0).pose_out
        self.laser_front_base_dist = self._distance([
                                                    laser_pose.pose.position.x, 
                                                    laser_pose.pose.position.y,
                                                    laser_pose.pose.position.z], 
                                                    [0,0,0])

        # Rear laser
        laser_pose = self.get_laser_rear_base_transform(0, 0).pose_out
        self.laser_rear_base_dist =  self._distance([
                                                    laser_pose.pose.position.x,
                                                    laser_pose.pose.position.y,
                                                    laser_pose.pose.position.z], 
                                                    [0,0,0])

        # Security tune-up variables
        self.max_rad = .5
        self.laser_range = pi / 9
        self.front_laser_dist = .25
        self.stoping_acc = 0.3
        # Subscriber variables
        self.curr_vel = 0
        self.sent_vel = 0

        # clock
        self.rate_pub = rospy.Rate(10)
        self.laser_front_cb_rate = rospy.Rate(5)
        self.laser_rear_cb_rate = rospy.Rate(5)
        self.cnt_front = 0
        self.cnt_rear = 0

        ### ROS interface ###
        # Velocity publisher
        self.pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=2)
        # Laser subscribers
        self.laser_front_sub = rospy.Subscriber('/bender/sensors/laser_front/scan', LaserScan, self.laser_front_input_cb, queue_size = 1)
        self.laser_rear_sub = rospy.Subscriber('/bender/sensors/laser_rear/scan', LaserScan, self.laser_rear_input_cb, queue_size = 1)
        # Sent velocity subscriber
        self.vel_sub = rospy.Subscriber("/bender/nav/low_level_mux/cmd_vel", Twist, self.vel_output_cb, queue_size = 1)
        # Odom subscriber
        self.odom_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self.odom_input_cb, queue_size = 1)

        self.marker = self.setup_marker()
        self.marker_pub = rospy.Publisher("/bender/markers/corrected_footprint", Marker, queue_size=1)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                # Obtaining distance from center of the closest point at the front and the back
                trans_front = self.laser_front_closest_point[0]
                rot_front = self.laser_front_closest_point[1]

                trans_rear = self.laser_rear_closest_point[0]
                rot_rear = self.laser_rear_closest_point[1]

                dist_front = self._distance([
                                            trans_front * cos(rot_front), 
                                            trans_front * sin(rot_front), 
                                            0], 
                                            [0,0,0])
                dist_rear  = self._distance([
                                            trans_rear * cos(rot_rear), 
                                            trans_rear * sin(rot_rear), 
                                            0], 
                                            [0,0,0])
                rospy.loginfo("dist front %f m, dist rear %f m" % (dist_front, dist_rear))

                # Chosing closest point between front and back
                closest = min(dist_rear, dist_front)
                if dist_front < dist_rear:
                    clos_ang = rot_front
                else:
                    clos_ang = rot_rear + pi

                # Calculating correction factor
                corr_factor = self.get_correction_factor(clos_ang)
                self.marker.scale.x = self.max_rad + corr_factor
                self.marker_pub.publish(self.marker)
                #rospy.loginfo("Closer point at %f m from the center with a %f correction_factor" % (closest, corr_factor))

                # Check if closest point is inside the safety area, in which case, stop movement if velocity moves the base in that direction
                if closest <= self.max_rad + abs(corr_factor) and corr_factor * self.sent_vel > 0:
                    rospy.loginfo("Collision detected, stopping movement")
                    # Publish empty Twist message to stop movement
                    self.pub.publish(Twist())
                # Sleep to mantain rate
                self.rate_pub.sleep()
        except Exception as e:
            rospy.logerr("Stopping safety controller. Because %s" % e)

    def _distance(self, pos1, pos2):
        """
        This method calculates the distance between two positions given in an array [x, y, z].

        Args:
            pos1 (list):
                first position
            pos2 (list):
                second position

        Returns:
            float: Distance between the two positions
        """
        dist = sqrt(mpow((pos1[0] - pos2[0]), 2) + mpow((pos1[1] - pos2[1]), 2) + mpow((pos1[2] - pos2[2]), 2))
        return dist

    def get_correction_factor(self, obj_rotation):
        """
        This method calculates the correction factor necesary for safety control, considering the current velocity and the stopping
        acceleration calculated by hand using the current Bender robot.

        Args:
            obj_rotation (float):
                Angle position of the closest obstacle

        Returns:
            float: Correction factor, > 0 if closest point is in the front, < 0 if it's in the back
        """
        vel_factor = mpow(max(abs(self.curr_vel), abs(self.sent_vel)), 2) / (2 * self.stoping_acc)
        ang_factor = 1 if cos(obj_rotation) > 0 else -1
        return vel_factor * ang_factor

    def get_laser_front_base_transform(self, dist, ang):
        """
        This method returns the transform between a point in the front laser coordinates to the base link.

        Args:
            dist (float): Distance from laser.
            ang (float): Angle from laser

        Returns:
            Transformer: Pose relative to base_link
        """

        pose = PoseStamped()
        pose.header.frame_id = self.laser_front_param

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)

        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]

        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame

        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def get_laser_rear_base_transform(self, dist, ang):
        """
        This method returns the transform between a point in the rear laser coordinates to the base link frame.

        Args:
            dist (float): Distance from laser.
            ang (float): Angle from laser.

        Returns:
            Transformer: Pose relative to base_link
        """
        pose = PoseStamped()
        pose.header.frame_id = self.laser_rear_param

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)

        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]

        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame

        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def laser_front_input_cb(self, msg):
        """
        This method is the callback function for the front laser subscriber. It calculates the distance and angle to the closest detected point.

        Args:
            msg (LaserScan): Laser scan message from front laser

        Returns:
            None
        """
        min_dist = float("inf")
        min_ang = pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        for i in range(2, len(msg.ranges)):
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            # Average three points at a time to eliminate false positives
            curr_mean = numpy.mean(curr_values)

            # Check if it's a valid point
            if msg.range_min <= curr_mean and curr_mean <= msg.range_max:
                curr_ang = msg.angle_min + i * msg.angle_increment

                # Get angle in base_link frame and check if it's in the valid range
                base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_front_base_dist + cos(curr_ang) * curr_mean)
                if abs(base_ang) < self.laser_range:
                    # Get distance in base_link frame and check if it's the minimum
                    curr_dist = sqrt(mpow(self.laser_front_base_dist, 2) + mpow(curr_mean, 2) + 2 * self.laser_front_base_dist * curr_mean * cos(curr_ang))
                    if curr_dist < min_dist:
                        min_ang = base_ang
                        min_dist = curr_dist
        # Update closest point variable
        self.laser_front_closest_point = [min_dist, min_ang]
        self.laser_front_cb_rate.sleep()

    def laser_rear_input_cb(self, msg):
        """
        This method is the callback function for the rear laser subscriber. It calculates the distance and angle to the closest detected point.

        Args:
            msg (LaserScan): Laser scan message from rear laser

        Returns:
            None
        """
        min_dist = float("inf")
        min_ang = pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        for i in range(2, len(msg.ranges)):
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            # Average three points at a time to eliminate false positives
            curr_mean = numpy.mean(curr_values)

            # Check if it's a valid point
            if msg.range_min <= curr_mean <= msg.range_max:
                curr_ang = msg.angle_min + i * msg.angle_increment

                # Get angle in base_link frame and check if it's in the valid range
                base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_rear_base_dist + cos(curr_ang) * curr_mean)
                if abs(base_ang) < self.laser_range:
                    # Get distance in base_link frame and check if it's the minimum
                    curr_dist = sqrt(mpow(self.laser_rear_base_dist, 2) + mpow(curr_mean, 2) + 2 * self.laser_rear_base_dist * curr_mean * cos(curr_ang))
                    if curr_dist < min_dist:
                        min_ang = base_ang
                        min_dist = curr_dist
        # Update closest point variable
        self.laser_rear_closest_point = [min_dist, min_ang]
        self.laser_rear_cb_rate.sleep()

    def odom_input_cb(self, msg):
        """
        This method is the callback function for the odometry subscriber.

        Args:
            msg (Odometry): Odometry message sent from the base.

        Returns:
            None
        """
        self.curr_vel = abs(msg.twist.twist.linear.x)
        self.rate_pub.sleep()

    def vel_output_cb(self, msg):
        """
        This method is the callback function for the sent velocity subscriber.

        Args:
            msg (Twist): Twist message sent by low_level_mux topic.

        Returns:
            None
        """
        self.sent_vel = msg.linear.x
        #self.rate_pub.sleep()

    def setup_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "bender/markers"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.max_rad
        marker.scale.y = self.max_rad
        marker.scale.z = 0.01
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker


def main():
    rospy.init_node('cmd_vel_low_level_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
