#!/usr/bin/env python3.9

#Std Libs
import numpy as np
np.float = np.float64
import math

#Ros Python Libs
import rospy
import roslib
import ros_numpy as rnp

#ROS msgs
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

#Image Processing
import cv2
import cv_bridge
from ultralytics import YOLO
from torchreid.utils import FeatureExtractor

def get_quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.
	
	Input
		:param roll: The roll (rotation around x-axis) angle in radians.
		:param pitch: The pitch (rotation around y-axis) angle in radians.
		:param yaw: The yaw (rotation around z-axis) angle in radians.
	
	Output
		:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	
	return [qx, qy, qz, qw]

class PersonLocator():
	def __init__(self):
		self.model = YOLO('yolov8n-pose.pt')
		rospy.loginfo('YOLOv8 is now running...')
		# self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
		# self.sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
		self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback)
		self.people_poses = rospy.Publisher('people_poses', PoseArray, queue_size=10)
		self.people_vector = rospy.Publisher('people_vector', Float64MultiArray, queue_size=10)
		#self.pose_pub = rospy.Publisher("/person_pose", PoseStamped, queue_size = 2)
		self.marker_pub = rospy.Publisher("/visualization_markers", MarkerArray, queue_size = 2)
		self.cv = cv_bridge.CvBridge()
		self._points_data = None
		self._image_data = None
		self.poses = PoseArray()
		self.poses.header.frame_id = "camera_depth_optical_frame"
		self.extractor = FeatureExtractor(
			model_name='mlfn',
			model_path='/home/pipe/Downloads/mlfn.pth.tar',
			device='cpu'
		)

	def callback(self,msg):
		seq = msg.header.seq
		try:
			self._points_data = rnp.numpify(msg)
			image_data = self._points_data['rgb'].view(
			(np.uint8, 4))[..., [0, 1, 2]]
			self._image_data = np.ascontiguousarray(image_data)
			# cv_image = self.cv.imgmsg_to_cv2(msg,'bgr8')
			# res = model(cv_image, show=True, conf=0.8)
			if not self._image_data is None:
				detections = self.model(self._image_data,show=False,conf=0.8)
			# print(res)
			marker_array = MarkerArray()
			self.poses.poses = []
			
			vector = Float64MultiArray()
			vector.data = []

			if len(detections)>0:
				d = detections[0]
				for i, (p, box) in enumerate(zip(d.keypoints.xy, d.boxes.data)):
					x, y, w, h = int(box[0].item()), int(box[1].item()), int(box[2].item()), int(box[3].item())

					ls_x, ls_y = (int(p[5,0].item()), int(p[5,1].item()))
					rs_x, rs_y = (int(p[6,0].item()), int(p[6,1].item()))
					lh_x, lh_y = (int(p[11,0].item()), int(p[11,1].item()))
					rh_x, rh_y = (int(p[12,0].item()), int(p[12,1].item()))
					person_x = int((ls_x+rs_x)/2)
					person_y = int((ls_y+rs_y)/2)

					if ls_x*rs_x:
						x1, x2 = min(ls_x,rs_x), max(ls_x,rs_x)
					else:
						x1, x2 = x, w

					if lh_y+rh_y:
						y2 = max(lh_y,rh_y,)
					else:
						y2 = h

					crop = image_data[y:y2,x1:x2]
					cv2.imshow('crop',crop)

					features = self.extractor(crop)[0]
					f = features.tolist()
					f = [float(value) for value in f]
					# xy = (person_x,person_y)
					# print(xy)
					# cv_image = np.copy(self._image_data)
					# cv_image = cv2.circle(cv_image,xy,radius=10,color=(0,0,255),thickness=-1)
					# cv2.imshow('img',cv_image)
					x = self._points_data['x'][person_y,person_x]
					y = self._points_data['y'][person_y,person_x]
					z = self._points_data['z'][person_y,person_x]

					body, head = self.target_marker(x, y, z, i)
					marker_array.markers.append(body)
					marker_array.markers.append(head)

					pose = Pose()
					# pose.header.frame_id = "d435_camera_depth_optical_frame"  # The source frame
					pose.position.x = x
					pose.position.y = y
					pose.position.z = z
					pose.orientation.w = 1
					self.poses.poses.append(pose)

					V = [float(x), float(y), float(z), *f]
					vector.data = [*vector.data, *V]

			print(np.array(vector.data).shape)
			self.people_poses.publish(self.poses)
			self.marker_pub.publish(marker_array)
			self.people_vector.publish(vector)
			
		except cv_bridge.CvBridgeError as e:
			print(e)
		cv2.putText(self._image_data,text = str(seq),org = (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (255,0,0), thickness=2)
		cv2.waitKey(3)

	def target_marker(self, x ,y, z, id, frame="/camera_depth_optical_frame"):
		##BODY
		body = Marker()

		body.header.frame_id = frame
		body.header.stamp = rospy.Time.now()

		# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
		body.type = 3
		body.id = (2*id)

		# Set the scale of the marker
		body.scale.x = 0.3
		body.scale.z = 0.4
		body.scale.y = 1.3

		# Set the color
		body.color.r = 0.0
		body.color.g = 1.0
		body.color.b = 0.0
		body.color.a = 0.75

		# Set the pose of the marker
		q = get_quaternion_from_euler(0,math.pi/2,0)

		body.pose.position.x = x
		body.pose.position.y = y + 0.6
		body.pose.position.z = z + 0.1
		body.pose.orientation.x = q[0]
		body.pose.orientation.y = q[1]
		body.pose.orientation.z = q[2]
		body.pose.orientation.w = q[3]

		##HEAD
		head = Marker()

		head.header.frame_id = frame
		head.header.stamp = rospy.Time.now()

		# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
		head.type = 2
		head.id = (2*id)+1

		# Set the scale of the marker
		head.scale.x = 0.3
		head.scale.y = 0.3
		head.scale.z = 0.3

		# Set the color
		head.color.r = 0.0
		head.color.g = 1.0
		head.color.b = 0.0
		head.color.a = 0.75

		# Set the pose of the marker
		head.pose.position.x = x
		head.pose.position.y = y - 0.175
		head.pose.position.z = z + 0.1
		head.pose.orientation.x = 0.0
		head.pose.orientation.y = 0.0
		head.pose.orientation.z = 0.0
		head.pose.orientation.w = 1.0

		return body, head


if __name__=='__main__':
	rospy.init_node('person_locator')
	person_locator = PersonLocator()
	rospy.spin()

