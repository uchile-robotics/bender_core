#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
import math
import numpy as np

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class OdometryHelper(object):
    """
    This class provides RVIZ markers for robot odometry tunning.

    It publishes cartesian and polar grids as rviz markers.
    """

    # COLORS
    COLOR_GRAY = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)
    COLOR_MAGENTA = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)

    def __init__(self):

        # =====================================================================
        # Parameters

        self.odom_frame = rospy.get_param("~odom_frame", "/bender/odom")
        self.dtheta = rospy.get_param("~dtheta", 1)  # [deg]
        self.min_polar_range = rospy.get_param("~min_polar_range", 0.4)  # [m]
        self.max_polar_range = rospy.get_param("~max_polar_range", 5.0)  # [m]
        self.x_size = rospy.get_param("~x_size", 10)   # [m]
        self.y_size = rospy.get_param("~y_size", 3)   # [m]
        self.dx = rospy.get_param("~dx", 0.05)   # [m]
        self.dy = rospy.get_param("~dy", 0.05)   # [m]
        self.line_width = rospy.get_param("~line_width", 0.0025)   # [m]
        self.spin_rate = rospy.Rate(10)

        # Topic Publishers
        self.marker_pub = rospy.Publisher("~grid", MarkerArray, queue_size=1)

    # =========================================================================
    # Main Processing method
    # =========================================================================

    def spin_once(self):
        msg = MarkerArray()
        now = rospy.get_rostime()
        msg.markers.append(self.gen_grid_cartesian(now))
        msg.markers.append(self.gen_grid_polar(now))
        self.marker_pub.publish(msg)
        self.spin_rate.sleep()

    # =========================================================================
    # Visualization Methods
    # =========================================================================

    def gen_grid_cartesian(self, time):
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = time
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "odometry/cartesian_grid"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color = OdometryHelper.COLOR_GRAY

        x_min = -self.x_size / 2.0
        x_max = self.x_size / 2.0
        y_min = -self.y_size / 2.0
        y_max = self.y_size / 2.0
        for x in np.arange(x_min, x_max, self.dx):
            marker.points.append(Point(x=x, y=y_min, z=0.01))
            marker.points.append(Point(x=x, y=y_max, z=0.01))
        for y in np.arange(y_min, y_max, self.dy):
            marker.points.append(Point(x=x_min, y=y, z=0.01))
            marker.points.append(Point(x=x_max, y=y, z=0.01))
        return marker

    def gen_grid_polar(self, time):
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = time
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "odometry/polar_grid"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color = OdometryHelper.COLOR_GRAY

        for theta in np.arange(-math.pi, math.pi, self.dtheta * math.pi / 180.0):
            x1 = self.min_polar_range * math.cos(theta)
            y1 = self.min_polar_range * math.sin(theta)
            x2 = self.max_polar_range * math.cos(theta)
            y2 = self.max_polar_range * math.sin(theta)
            marker.points.append(Point(x=x1, y=y1, z=0.01))
            marker.points.append(Point(x=x2, y=y2, z=0.01))
        return marker


def main():
    rospy.init_node('cmd_vel_low_level_safety')
    handler = OdometryHelper()
    while not rospy.is_shutdown():
        handler.spin_once()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Stopping node due to rospy shutdown signal.")
