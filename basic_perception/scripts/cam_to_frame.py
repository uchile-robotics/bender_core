#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

def pose_callback(pose_msg):
    try:
        # Initialize the tf2 listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Wait for the transform from /person_pose to /camera_link
        transform = tf_buffer.lookup_transform("bender/base_link", pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

        # Transform the pose to /camera_link frame
        transformed_pose = do_transform_pose(pose_msg, transform)

        # Update the frame_id to /camera_link
        transformed_pose.header.frame_id = "bender/base_link"
        transformed_pose.pose.position.z = -0.7
        transformed_pose.pose.orientation.x = 0.0
        transformed_pose.pose.orientation.y = 0.0
        transformed_pose.pose.orientation.z = 0.0
        transformed_pose.pose.orientation.w = 1.0

        # Publish the transformed pose
        person_location_publisher.publish(transformed_pose)

    except Exception as e:
        rospy.logerr("Transform error: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    # Initialize the subscriber to the /person_pose topic
    rospy.Subscriber("/person_pose", PoseStamped, pose_callback)

    # Initialize the publisher for /person_location topic
    person_location_publisher = rospy.Publisher("/person_location", PoseStamped, queue_size=10)

    rospy.spin()