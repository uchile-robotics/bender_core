#!/usr/bin/env python
"""
This node is just a proxy for LaserScan generated from stage.

It just changes the 'header.frame_id' value of incoming scans
"""
import rospy
import sys
from sensor_msgs.msg import LaserScan

pub = None
frame_id = None


def callback(scan):
    global pub, frame_id
    scan.header.frame_id = frame_id
    pub.publish(scan)


def proxy():
    global pub, frame_id
    rospy.init_node('laser_scan_proxy')

    try:
        frame_id = rospy.get_param("~frame_id")
    except KeyError:
        rospy.logerr("The frame_id parameter is not set!.")
        sys.exit(1)

    # define publisher
    pub = rospy.Publisher('output_scan', LaserScan, queue_size=10)

    # use defined publisher on callback
    rospy.Subscriber("input_scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        proxy()
    except rospy.ROSInterruptException:
        pass
