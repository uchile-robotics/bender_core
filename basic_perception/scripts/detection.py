#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import os
import bleedfacedetector as fd
import time
import emotion_detect as ed
import predict_age as pa
import predict_gender as pg

class detect_person:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
    self.emotion_pub = rospy.Publisher("/emotion_detect", String, queue_size=10)
    self.genre_pub = rospy.Publisher("/predict_gender", String, queue_size=10)
    self.age_pub = rospy.Publisher("/predict_age", String, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/bender/sensors/rgbd_head/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img_copy = cv_image.copy()
    
    # Detect faces in image
    faces = fd.ssd_detect(img_copy,conf=0.2)
    
    face_close = faces[0] 
    x = face_close[0]
    y = face_close[1]
    w = face_close[2]
    h = face_close[3]
    output_emotion = String()
    output_age = String()
    output_genre = String()
    ed.init_emotion('/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/basic_perception/scripts/emotion-ferplus-8.onnx') 
    emotion = ed.emotion(cv_image, returndata=True)
    age = pa.predict_age(x,y,w,h,img_copy)
    genre = pg.predict_gender(x,y,w,h,img_copy)
    output_emotion.data = emotion[1]
    output_age.data = age
    output_genre.data = genre
    self.emotion_pub.publish(output_emotion)
    self.genre_pub.publish(output_age)
    self.age_pub.publish(output_genre)

def main(args):
  ic = detect_person()
  rospy.init_node('detect_person', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)