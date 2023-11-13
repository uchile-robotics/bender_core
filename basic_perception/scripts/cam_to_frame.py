#!/usr/bin/env python

import rospy
import tf2_ros
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

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    # Initialize the subscriber to the /person_pose topic
    rospy.Subscriber("/people_poses", PoseArray, pose_callback)

    # Initialize the publisher for /person_location topic
    people_location_publisher = rospy.Publisher("/people_locations", PoseArray, queue_size=10)

    rospy.spin()