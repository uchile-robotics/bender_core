#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
#from centro_2 import get_quaternion_from_euler, find_object_by_class
import json

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

def coords_callback(coords_msg):
    try:
        # Initialize the tf2 listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        marker_pub = rospy.Publisher("/visualization_markers", MarkerArray, queue_size = 2)

        # Wait for the transform from /person_pose to /camera_link
        # Hay que cambiar el bender/odom y camera_color...
        transform = tf_buffer.lookup_transform("bender/odom", "d435_camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))

        # se usa json.loads
        object_data = json.loads(coords_msg.data)
        marker_array = MarkerArray()
        for obj in object_data:
            obj_class_n = obj['class'][0]  # Obtiene el nombre de la clase del objeto
            x = obj['x']
            y = obj['y']
            z = obj['z']

            ps = PointStamped()
            #ps.header.stamp = get_clock().now().to_msg()
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z

            print(ps)

            # Transform the pose to /camera_link frame
            transformed_point = do_transform_point(ps, transform)
            print(transformed_point)

            ## se publica el punto transformado
            #point_location_publisher.publish(transformed_point)


            # Hacemos markers
            object_i = Marker()

            object_i.header.frame_id = "bender/odom"
            object_i.header.stamp = rospy.Time.now()
            object_i.action = 0

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            object_i.type = 2
            object_i.id = int(obj_class_n)

            # Set the scale of the marker
            object_i.scale.x = 0.1
            object_i.scale.z = 0.1
            object_i.scale.y = 0.1

            # Set the color
            object_i.color.r = 0.0
            object_i.color.g = 1.0
            object_i.color.b = 0.0
            object_i.color.a = 1.0

            # Set the pose of the marker
            q = get_quaternion_from_euler(0,0,0)

            object_i.pose.position.x = transformed_point.point.x
            object_i.pose.position.y = transformed_point.point.y
            object_i.pose.position.z = transformed_point.point.z
            object_i.pose.orientation.x = q[0]
            object_i.pose.orientation.y = q[1]
            object_i.pose.orientation.z = q[2]
            object_i.pose.orientation.w = q[3]

            #object_i.lifetime = 1

            # se agregan markers para cada objeto


            marker_array.markers.append(object_i)
        
        # Se publica un array de markers
        print(type(marker_array))
        marker_pub.publish(marker_array)

        # Publish the transformed pose
        

    except Exception as e:
        rospy.logerr("Transform error: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    # Initialize the subscriber to the /person_pose topic
    rospy.Subscriber("/object_info", String, coords_callback)

    # Initialize the publisher for /person_location topic

    #point_location_publisher = rospy.Publisher("/object_location", PointStamped, queue_size=10)

    # Para publicar markers
    marker_pub = rospy.Publisher("/visualization_markers", MarkerArray, queue_size = 2)

    rospy.spin()