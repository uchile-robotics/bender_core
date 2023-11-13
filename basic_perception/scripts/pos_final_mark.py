#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
#import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
import json
import random


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

# try:
#     with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', 'r') as objetos_json:
#         data_nom = json.load(objetos_json)
# except ValueError as e:
#     if "No JSON object could be decoded" in str(e):
#         # Manejar el caso de JSON vacío, por ejemplo, asignar una lista vacía a data_nom
#         data_nom = []
#     else:
#         # Manejar otros errores de decodificación JSON
#         print(f"Error de decodificación JSON: {e}")
# except Exception as e:
#     # Manejar otras excepciones
#     print(f"Ocurrió un error: {e}")


try:
    with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', 'r') as objetos_json:
        data_nom = json.load(objetos_json)
except ValueError as e:
    if "No JSON object could be decoded" in str(e):
        # Manejar el caso de JSON vacío, por ejemplo, asignar una lista vacía a data_nom
        data_nom = []
    else:
        # Manejar otros errores de decodificación JSON
        print("Error de decodificación JSON: {}".format(e))
except Exception as e:
    # Manejar otras excepciones
    print("Ocurrió un error: {}".format(e))
# with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', 'r') as objetos_json:
#     data_nom = json.load(objetos_json)

class Posicion_objetos():
    def __init__(self):
        self.objects = []  # Almacena las últimas posiciones de objetos
        #self.last_positions = {}
        self.alpha = 0.25

        # Suscripción al nodo que publica posiciones detectadas en formato JSON
        self.sub = rospy.Subscriber('/object_info_t', String, self.callback)
        self.pub = rospy.Publisher('/object_info_t', String, queue_size=10)

    def mismo_objeto(self,obj1,obj2,umbral):
        dist = np.sqrt((obj2['x'] - obj1['x'])**2 + (obj2['y'] - obj1['y'])**2 + (obj2['z'] - obj1['z'])**2)
        if obj1['class'] == obj2['class'] and dist <= umbral:
            return True
        else:
            return False
    
    
    # def agregar_posicion(self,obj_id, posicion):
    #     if obj_id not in self.last_positions:
    #         if self.mismo_objeto()
    #         self.last_positions[obj_id] = []  # Crea una lista vacía para el objeto si es la primera vez
    #     self.last_positions[obj_id].append(posicion)
        
    #     # Limita la lista a las últimas 5 posiciones
    #     if len(self.last_positions[obj_id]) > 5:
    #         self.last_positions[obj_id].pop(0)  # Elimina la posición más antigua
    
    def callback(self,msg):
        # try:
        object_data = json.loads(msg.data)

        for id, obj in enumerate(object_data):
        
            obj_class_n = obj['class']
            x = obj['x']
            y = obj['y']
            z = obj['z']

            if len(data_nom)==0:
                with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', "w") as posiciones_json:
                    json.dump(obj, posiciones_json, indent=2)
            else:
                for obj_json in data_nom:
                    if self.mismo_objeto(obj,obj_json,0.01):
                        x_n = self.alpha*x + (1-self.alpha)*obj_json['x']
                        y_n = self.alpha*y + (1-self.alpha)*obj_json['y']
                        z_n = self.alpha*z + (1-self.alpha)*obj_json['z']
                        obj_act = {
                            "class": obj_class_n,
                            "x": x_n,
                            "y": y_n,
                            "z": z_n
                        }
                        obj_json = obj_act
                        with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', "w") as posiciones_json:
                            json.dump(obj_json, posiciones_json, indent=2)
                    else:
                        with open('/home/bruno/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/posiciones_rs.json', "w") as posiciones_json:
                            json.dump(obj, posiciones_json, indent=2)
        # except:
        #     print("error")

class Json_a_marker():
    def __init__(self):
        self.marker_pub = rospy.Publisher("/markers_posicion", MarkerArray, queue_size = 2)
        self.marker_array = MarkerArray()
    
    def callback(self):
        for obj in data_nom:
            print(obj)
            # Hacemos markers
            object_i = Marker()
            
            # el frame_id puede estar malo
            object_i.header.frame_id = "d435_camera_depth_optical_frame"
            object_i.header.stamp = rospy.Time.now()
            object_i.action = 0

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            object_i.type = 2
            object_i.id = int(obj['class'])

            # Set the scale of the marker
            object_i.scale.x = 0.1
            object_i.scale.z = 0.1
            object_i.scale.y = 0.1

            # Set the color ## cómo lo hago para que sean distintos
            object_i.color.r = 0.0
            object_i.color.g = 1.0
            object_i.color.b = 0.0
            object_i.color.a = 1.0

            # Set the pose of the marker
            q = get_quaternion_from_euler(0,0,0)

            # Verificar si así se accede a las coords de los puntos transformados
            object_i.pose.position.x = obj["x"]
            object_i.pose.position.y = obj["y"]
            object_i.pose.position.z = obj["z"]
            object_i.pose.orientation.x = q[0]
            object_i.pose.orientation.y = q[1]
            object_i.pose.orientation.z = q[2]
            object_i.pose.orientation.w = q[3]

            #object_i.lifetime = 1

            # se agregan markers para cada objeto
            self.marker_array.markers.append(object_i)
        print(type(self.marker_array))
        self.marker_pub.publish(self.marker_array)

        


if __name__ == '__main__':
    rospy.init_node('Posicion_objetos', anonymous=True)
    posicion_final = Posicion_objetos()
    posicion_marker = Json_a_marker()
    rospy.spin()

# este .py entrega un json con las posiciones finales, pero no son gaussianas :(
                            



                            


                            











