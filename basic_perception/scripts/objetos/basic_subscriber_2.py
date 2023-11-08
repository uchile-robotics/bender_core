#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import json


def callback(coords_msg):
    object_data = json.loads(coords_msg.data)
    try:
        for obj in object_data:
            obj_class, obj_class_n = obj['class']  # Obtiene el nombre de la clase del objeto
            x = obj['x']
            y = obj['y']
            z = obj['z']

            ps = PointStamped()
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z

            # Transform the pose to /camera_link frame
            transformed_point = do_transform_point(ps, transform)
            print(ps)
    except:
        print('No se han encontrado objetos.')
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("object_info", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()