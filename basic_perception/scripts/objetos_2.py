#!/usr/bin/env python3.8

# Std Libs
import numpy as np
import math
np.float = np.float64

# ROS Python Libs
import rospy
import roslib
import ros_numpy as rnp
import sensor_msgs.point_cloud2 as pc2

# ROS msgs
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

# Image Processing
import cv2
from ultralytics    import YOLO

from cv_bridge import CvBridge, CvBridgeError


def flatten(xss):
    return [x for xs in xss for x in xs]

class ObjectLocator():
    def __init__(self):
        self.model = YOLO('yolov8n.pt')  # Load YOLOv8 model
        rospy.loginfo('YOLOv8 is now running...')
        
        # Subscribe to the RGB camera image and point cloud
        self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback)
        
        # Publisher for YOLO detection results
        self.yolo_pub = rospy.Publisher('yolo_pub', Image, queue_size=10)
        self.select_pub = rospy.Publisher('select_pub', PointCloud2, queue_size=10)
        
        self._image_data = None
        self._depth_image_data = None
        self._points_data = None
        self.bridge = CvBridge()

        self.list_xy = []
        self.list_xyz = []
        self.list_sin_nan = []
        
        
    def callback(self, msg: PointCloud2):
        # Convert ROS image to numpy array
        self._points_data = rnp.numpify(msg)  # Convert the message to a numpy array
        image_data = self._points_data['rgb'].view((np.uint8, 4))[..., [0, 1, 2]]
        self._image_data = np.ascontiguousarray(image_data)
        image_height, image_width = self._image_data.shape[:2]

        # Proceed with detection if the image data is valid
        if self._image_data is not None:
            # Perform object detection on the image
            results = self.model(self._image_data, show=True, conf=0.1)

            # Iterate through the results and extract bounding boxes
            for result in results:
                boxes = result.boxes  # Get the bounding boxes from the result

            full_cloud = []
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates [x1, y1, x2, y2]
                
                if y2 > 400: #x1, y1 Top-left corner
                    continue

                #Draw bounding box (OpenCV)
                cv2.rectangle(self._image_data, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
                # Convertimos los puntos en un array de numpy
                points_array = np.array(list(cloud_points))
                points_array = points_array.reshape((image_height, image_width, 3))
                
                
                sub_image_cloud = points_array[int(y1):int(y2), int(x1):int(x2)]
                
                sub_image_cloud = sub_image_cloud.reshape(-1, 3)
                
                
                #Nueva nube
                full_cloud.append(sub_image_cloud)
    
            # print(full_cloud)
            valid_xyz_array = np.array(flatten(full_cloud))
            # print('aaaaa')
            # print((valid_xyz_array))
            new_cloud_msg = pc2.create_cloud_xyz32(msg.header, valid_xyz_array)
            
            #Publish the processed image with YOLO bounding boxes
            self.select_pub.publish(new_cloud_msg)









                        

                        


            



            





        # #AAAA

        # # Convertir el mensaje PointCloud2 a un array de numpy
        # # Solo extraemos los campos 'x', 'y' y 'z'
        # cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # # Convertimos los puntos en un array de numpy
        # points_array = np.array(list(cloud_points))

        # # Reshape points_array to have each point as a row with (x, y, z) columns
        # # Assuming the cloud contains 3 values per point (x, y, z), we reshape to (-1, 3)
        # points_array = points_array.reshape(-1, 3)

        # # Ahora 'points_array' es un array donde cada fila es un punto en (x, y, z)
        # x = points_array[:, 0]  # Coordenadas x
        # y = points_array[:, 1]  # Coordenadas y
        # z = points_array[:, 2]  # Coordenadas z


        # image_data = self._points_data['rgb'].view((np.uint8, 4))[..., [0, 1, 2]]
        # self._image_data = np.ascontiguousarray(image_data)
        # self.img = rnp.numpify(msg)
        # # Perform object detection using YOLO
        # results = self.model(self.img, show=True, conf=0.1)  # `conf` is the confidence 
        

        # # Iterate through the results and extract bounding boxes
        # for result in results:
        #     boxes = result.boxes  # Get the bounding boxes from the result

        # for box in boxes:
        #     x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates [x1, y1, x2, y2]
            
                
        # #Draw bounding box (OpenCV)
        # cv2.rectangle(self.img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        # cv2.putText(self.img, f"ID: {class_id} Conf: {confidence:.2f}", 
        # (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # #Extract region of interest (ROI) based on bounding box coordinates
        # x1, y1 = box.xyxy[0][:2]  # Top-left corner
        # x2, y2 = box.xyxy[0][2:]  # Bottom-right corner
        # sub_image = self.img[int(y1):int(y2), int(x1):int(x2)]
        # height, width = sub_image.shape[:2]
                

        # #Iterate through each pixel in the sub-image and record its coordinates
        # for y in range(height):
        #     for x in range(width):
        #         self.list_xy.append([x, y]) #Lista extraida de caja delimitadora con x e y
                    
               

        # #Convert the processed image back to ROS message format
        # self.img_ros = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
                
        # #Publish the processed image with YOLO bounding boxes
        # self.yolo_pub.publish(self.img_ros)


if __name__ == '__main__':
    rospy.init_node('object_locator')
    object_locator = ObjectLocator()
    rospy.spin()