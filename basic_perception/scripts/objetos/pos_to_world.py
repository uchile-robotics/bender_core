#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
from basic_perception.msg import ObjectInfo
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
        transform_list = []
        for obj in object_data:
            obj_class_n = obj['class']  # Obtiene el nombre de la clase del objeto

            ps = PointStamped()
            ps.point.x = obj['x']
            ps.point.y = obj['y']
            ps.point.z = obj['z']

            # Transform the pose to /camera_link frame
            transformed_point = do_transform_point(ps, transform)
            print(transformed_point)

            info_objeto = {
              "ID": , 
              "class": int(obj_class_n), 
              "x" : x.item(), 
              "y" : y.item(), 
              "z" : z.item()}
            transform_list.append(info_objeto)
        for obj_t in transform_list:
          
        

        
        # Se publica 
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