#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import json

def coords_callback(self,coords_msg):
    try:
        # Initialize the tf2 listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Wait for the transform from /person_pose to /camera_link
        transform = tf_buffer.lookup_transform("bender/nav/odom", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))


        object_data = json.loads(coords_msg.data)
        for obj in object_data:
            obj_class, obj_class_n = obj['class']  # Obtiene el nombre de la clase del objeto
            x = obj['x']
            y = obj['y']
            z = obj['z']

            ps = PointStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z

            # Transform the pose to /camera_link frame
            transformed_point = do_transform_point(ps, transform)

            point_location_publisher.publish(transformed_point)

        # Publish the transformed pose
        

    except Exception as e:
        rospy.logerr("Transform error: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    # Initialize the subscriber to the /person_pose topic
    rospy.Subscriber("/object_info", PointStamped, coords_callback)

    # Initialize the publisher for /person_location topic
    point_location_publisher = rospy.Publisher("/object_location", PointStamped, queue_size=10)

    rospy.spin()