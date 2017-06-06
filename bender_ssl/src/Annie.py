#!/usr/bin/env python

import rospy
import math
import numpy as n
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource, HarkSourceVal, promxy
from std_msgs.msg import Float32

class MicroconeMark(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/talking_topic', promxy, self.marker_callback)
		self.pub = rospy.Publisher('/annie_topic', Marker, queue_size=10)
		self.pub2 = rospy.Publisher('/annie_angle', Float32, queue_size=10)
		self.marker = Marker()
		self.angle = 0.0

	def marker_callback(self, msg):
		shape = 2
		self.angle = n.arctan2(msg.y,msg.x)
		if self.angle >= 0:
			self.angle = self.angle*180.0/n.pi 
		else:
			self.angle = ((2*n.pi)+self.angle)*180.0/n.pi
		self.pub2.publish(self.angle)
		self.marker.header.frame_id = "baselim"
		self.marker.ns = "test"
		self.marker.id = 0
		self.marker.type = shape
		self.marker.action = 0
		self.marker.pose.position.x = msg.x
		self.marker.pose.position.y = msg.y
		self.marker.pose.position.z = 0.0
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5
		self.marker.color.r = 255
		self.marker.color.g = 0.0
		self.marker.color.b = 255
		self.marker.color.a = 1.0
		self.marker.lifetime = rospy.Duration(10)
		#rospy.loginfo(self.marker.pose.position.x)
		self.pub.publish(self.marker)

def main():
	rospy.init_node('base_controlleran')
	rospy.loginfo('Init base controller')
	base = MicroconeMark()   
	rospy.spin()

if __name__ == '__main__':
	main()