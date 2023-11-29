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

class emotion_detect:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
    self.emotion_pub = rospy.Publisher("/emotion_detect", String, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    fps=0
    output = String()

    #LINK EMOTION-FERPLU-8: https://github.com/onnx/models/tree/main/vision/body_analysis/emotion_ferplus/model
    ed.init_emotion('/home/bender/uchile_ws/pkgs/base_ws/bender_core/basic_perception/scripts/weights/emotion-ferplus-8.onnx') 
    start_time = time.time()
    emotion = ed.emotion(cv_image, returndata=True)
    cv_image = emotion[0]
    output.data = emotion[1]
    self.emotion_pub.publish(output)
    fps= (1.0 / (time.time() - start_time))
    cv2.putText(cv_image, 'FPS: {:.2f}'.format(fps), (10, 20), cv2.FONT_HERSHEY_SIMPLEX,0.8, (255, 20, 55), 1)
    cv2.imshow("Emotion Recognition",cv_image)
    
    cv2.waitKey(3)
        
    

def main(args):
  ic = emotion_detect()
  rospy.init_node('emotion_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)