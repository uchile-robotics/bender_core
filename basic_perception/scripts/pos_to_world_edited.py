#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
#from basic_perception.msg import ObjectInfo
import json


class Nodo_transformada():
    def __init__(self):
        self.sub = rospy.Subscriber("/object_info", String, self.callback)
        self.pub = rospy.Publisher('/object_info_t', String, queue_size=10)
        self.lista_transformada = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = self.tf_buffer.lookup_transform("bender/odom", "d435_camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
        
    
    def callback(self,msg):
        try:
            object_data = json.loads(msg.data)
            for obj in object_data:
                obj_class_n = obj['class']  # Obtiene el nombre de la clase del objeto

                ps = PointStamped()
                ps.point.x = obj['x']
                ps.point.y = obj['y']
                ps.point.z = obj['z']

                # Transform the pose to /camera_link frame
                transformed_point = do_transform_point(ps, self.transform)
                print(transformed_point.point.x)

                obj_transformado = {
                "class": int(obj_class_n), 
                "x" : transformed_point.point.x, 
                "y" : transformed_point.point.y, 
                "z" : transformed_point.point.z}
                
                self.lista_transformada.append(obj_transformado)
            self.pub.publish(json.dumps(self.lista_transformada))
            self.lista_transformada = []
        except Exception as e:
            rospy.logerr("Transform error: %s", str(e))
         

if __name__ == '__main__':
    rospy.init_node('nodo_transformada')
    transformador_de_posiciones = Nodo_transformada()
    rospy.spin()


# este .py se hace aparte porque tf2 necesita python2 para correr creo