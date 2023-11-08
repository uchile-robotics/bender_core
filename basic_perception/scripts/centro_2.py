#!/usr/bin/env python3

#Std Libs
import numpy as np
import math
np.float = np.float64

#Ros Python Libs
import rospy
import roslib
import ros_numpy as rnp

#ROS msgs
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

#Image Processing
import cv2
import cv_bridge
from ultralytics import YOLO

#JSON Files
import json

lista_posiciones = []

# Json para extraer características y nombres de objetos
with open('/home/bender/gpsr_ws/src/vision_opencv/opencv_tests/nodes/objetos_2.json', 'r') as objetos_json:
    data_nom = json.load(objetos_json)

# Función que encuentra objeto por su clase
def find_object_by_class(json_data, target_class):
    # Recorre todas las categorías y sus objetos
    for category in json_data['categories']:
        if 'objects' in category:
            for obj in category['objects']:
                if obj.get('class') == target_class:
                    return obj,category['name']  # Retorna el objeto si encuentra la clase

    return None, None  # Retorna None si no encuentra la clase en el JSON

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

class_list = [25,26,27,28,29,33,40,41,42,43,44,45,46,47,48,49,50,51,52,53,
			  54,55,56,57,58,59,61,63,64,65,66,67,68,69,72,73,74,75,76,77,78,79,80]


class Nodo_test():
    def __init__(self):
        self.model = YOLO('yolov8m.pt').to('cuda')
        rospy.loginfo('YOLOv8 is now running...')
        # self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback)
        self.pub = rospy.Publisher('object_info', String, queue_size=10)
        #self.person_poses = rospy.Publisher('person_pose', PoseStamped, queue_size=10)
        #self.marker_pub = rospy.Publisher("/visualization_markers", MarkerArray, queue_size = 2)
        self.cv = cv_bridge.CvBridge()
        self.lista_posiciones = []
        self._points_data = None
        self._image_data = None
        rospy.init_node('Nodo_test', anonymous=True)
        #self.marker_array_1 = MarkerArray()

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
                detections = self.model.track(self._image_data, show=True, persist=True, classes = class_list, conf=0.8)
            marker_array = MarkerArray()
            if len(detections)>0:
                for i, d in enumerate(detections):
                    boxes_i = d.boxes.data
                    for b in boxes_i:
                        x_b,y_b,w_b,h_b = b[0:4]
                        x = self._points_data['x'][int(y_b),int(x_b)]
                        print('x: '+str(x))
                        y = self._points_data['y'][int(y_b),int(x_b)]
                        z = self._points_data['z'][int(y_b),int(x_b)]
                        n_class = d.boxes.cls.cpu().numpy().astype(int)+1
                        
                        # ARREGLAR QUE ESTÁ TIRANDO LA MISMA UBICACIÓN PARA TODOS LOS OBJETOSOSSSS
                        obj, cat = find_object_by_class(data_nom,n_class[0])
                        if obj is not None:
                            info_objeto = {
                                "ID": i,
                                "class": [int(n_class[0]),obj['name']],
                                "x" : x.item(),
                                "y" : y.item(),
                                "z" : z.item()
                            }
                        self.lista_posiciones.append(info_objeto)
                    
                self.pub.publish(json.dumps(self.lista_posiciones))     # se guarda como json.dumps, se lee como json.loads
                self.lista_posiciones = []

            #print(cv_image.shape)
        except cv_bridge.CvBridgeError as e:
            print(e)
        cv2.putText(self._image_data,text = str(seq),org = (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (255,0,0), thickness=2)
        #cv2.imshow('hola',cv_image)
        cv2.waitKey(3)

if __name__=='__main__':
   # rospy.init_node('yolo')
    test = Nodo_test()
    rospy.spin()

# Json para escribir ubicaciones de objetos

