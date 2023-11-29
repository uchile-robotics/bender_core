#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf2_geometry_msgs import do_transform_pose

def pose_callback(pose_msg):
	try:
		# Initialize the tf2 listener
		tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		target_frame = "camera_link"
		# target_frame = "bender/base_link"

		# Wait for the transform from /person_pose to /camera_link
		transform = tf_buffer.lookup_transform(target_frame, pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
		poses = PoseArray()
		poses.header.frame_id = target_frame
		poses.poses = []

		for p in pose_msg.poses:
			# Transform the pose to /camera_link frame
			current_pose = PoseStamped()
			current_pose.header.frame_id = pose_msg.header.frame_id
			current_pose.pose = p
			transformed_pose = do_transform_pose(current_pose, transform)

			# Update the frame_id to /camera_link
			transformed_pose.header.frame_id = target_frame
			transformed_pose.pose.position.z = -0.7
			transformed_pose.pose.orientation.x = 0.0
			transformed_pose.pose.orientation.y = 0.0
			transformed_pose.pose.orientation.z = 0.0
			transformed_pose.pose.orientation.w = 1.0

			poses.poses.append(transformed_pose.pose)

		# Publish the transformed pose
		people_location_publisher.publish(poses)

	except Exception as e:
		rospy.logerr("Transform error: %s", str(e))

def vector_callback(msg):
	try:
		# Initialize the tf2 listener
		tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		current_frame = "camera_depth_optical_frame"
		target_frame = "camera_link"
		# target_frame = "bender/base_link"

		# Wait for the transform from /person_pose to /camera_link
		transform = tf_buffer.lookup_transform(target_frame, current_frame, rospy.Time(0), rospy.Duration(1.0))
		poses = PoseArray()
		poses.header.frame_id = target_frame
		poses.poses = []

		vector = Float64MultiArray()
		vector.data = []

		lst = msg.data
		n_detections = len(lst) // 1027
		reconstructed = []
		#Reconstruct Matrix
		for i in range(n_detections):
			first_idx = i*1027
			last_idx = first_idx+1027
			reconstructed.append(lst[first_idx:last_idx])

		print np.array(reconstructed).shape

		for v in reconstructed:
			# Transform the pose to /camera_link frame
			current_pose = PoseStamped()
			current_pose.header.frame_id = current_frame
			current_pose.pose.position.x = v[0]
			current_pose.pose.position.y = v[1]
			current_pose.pose.position.z = v[2]
			current_pose.pose.orientation.w = 1.0
			transformed_pose = do_transform_pose(current_pose, transform)


			# Update the frame_id to /camera_link
			transformed_pose.header.frame_id = target_frame
			transformed_pose.pose.position.z = -0.7
			transformed_pose.pose.orientation.x = 0.0
			transformed_pose.pose.orientation.y = 0.0
			transformed_pose.pose.orientation.z = 0.0
			transformed_pose.pose.orientation.w = 1.0

			poses.poses.append(transformed_pose.pose)

			x = transformed_pose.pose.position.x
			y = transformed_pose.pose.position.y

			vector.data = list(vector.data) + [x, y] + list(v[3:])

		# Publish the transformed pose
		people_location_publisher.publish(poses)
		people_vector_publisher.publish(vector)
		print(len(vector.data))

	except Exception as e:
		rospy.logerr("Transform error: %s", str(e))

if __name__ == '__main__':
	rospy.init_node('pose_transformer_node')

	# Initialize the subscriber to the /person_pose topic
	# rospy.Subscriber("/people_poses", PoseArray, pose_callback)
	rospy.Subscriber("/people_vector", Float64MultiArray, vector_callback)

	# Initialize the publisher for /person_location topic
	people_location_publisher = rospy.Publisher("/people_locations", PoseArray, queue_size=10)
	people_vector_publisher = rospy.Publisher("/people_detections", Float64MultiArray, queue_size=10)

	rospy.spin()