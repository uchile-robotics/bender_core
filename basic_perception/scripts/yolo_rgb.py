#!/usr/bin/env python3.9

#Std Libs
import numpy as np
import math
np.float = np.float64

#Ros Python Libs
import rospy
import roslib
import ros_numpy as rnp

#ROS msgs
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

#Image Processing
# import cv2
from ultralytics import YOLO

class PersonLocator():
	def __init__(self):
		self.model = YOLO('yolov8n-pose.pt')
		rospy.loginfo('YOLOv8 is now running...')
		# self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
		# self.sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
		self.yolo_pub = rospy.Publisher('yolo_pub', Image, queue_size=10)
		#self.pose_pub = rospy.Publisher("/person_pose", PoseStamped, queue_size = 2)
		self._image_data = None

	def callback(self,msg):
		img = rnp.numpify(msg)
		res = self.model(img,show=True,conf=0.5)

		self.yolo_pub.publish(msg)


if __name__=='__main__':
	rospy.init_node('person_locator')
	person_locator = PersonLocator()
	rospy.spin()

