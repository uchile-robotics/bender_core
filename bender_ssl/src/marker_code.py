#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource, HarkSourceVal, promxy
from std_msgs.msg import Float32

class MicroconeMark(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/hark_source', HarkSource, self.marker_callback)
        self.sub2 = rospy.Subscriber('prom_topic', promxy, self.marker2_callback)
        self.pub2 = rospy.Publisher('prom_marker_topic', Marker, queue_size=10)
        self.pub = rospy.Publisher('microcone_topic', Marker, queue_size=10)
        self.marker = Marker()
        self.marker2 = Marker()

    def marker_callback(self, msg):
        shape = 1
        self.marker.header.frame_id = "baselim"
        self.marker.ns = "test"
        self.marker.id = 0
        self.marker.type = shape
        self.marker.action = 0
        self.marker.pose.position.x = msg.src[0].x
        self.marker.pose.position.y = msg.src[0].y
        self.marker.pose.position.z = msg.src[0].z
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 255
        self.marker.color.g = 255
        self.marker.color.b = 255
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration(10)
        #rospy.loginfo(self.marker.pose.position.x)
        self.pub.publish(self.marker)

    def marker2_callback(self, msg):
        shape = 1
        self.marker2.header.frame_id = "baselim"
        self.marker2.ns = "test"
        self.marker2.id = 1
        self.marker2.type = shape
        self.marker2.action = 0
        self.marker2.pose.position.x = msg.x
        self.marker2.pose.position.y = msg.y
        self.marker2.pose.position.z = 0.0
        self.marker2.scale.x = 0.5
        self.marker2.scale.y = 0.5
        self.marker2.scale.z = 0.5
        self.marker2.color.r = 0
        self.marker2.color.g = 255
        self.marker2.color.b = 255
        self.marker2.color.a = 1.0
        self.marker2.lifetime = rospy.Duration(10)
        #rospy.loginfo(self.marker2.pose.position.x)
        print "publicando"
        self.pub2.publish(self.marker2)

def main():
    rospy.init_node('base_controller')
    rospy.loginfo('Init base controller')
    base = MicroconeMark()   
    rospy.spin()

if __name__ == '__main__':
    main()