#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('cv_bridge')

import sys
import rospy
import cv2
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import bleedfacedetector as fd
import numpy as np

class imageConverter(object):

  def __init__(self, frame_patience=5):
    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("/bender/sensors/rgbd_head/rgb/image_rect_color", Image, self.callback)
    
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
    self._coords_pub = rospy.Publisher('/detected_face/coords', Int32MultiArray, queue_size=10)
    
    self.face_coords = Int32MultiArray()

    # coords to send when no face is detected
    self.NO_FACE = [-99, -99]

    # Number of frames with no detection to consider as no face present
    self.frame_patience = frame_patience
    self.no_face_counter = 0
    
  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    faces = np.array(fd.ssd_detect(cv_image, conf=0.75, returnconf=False))

    if faces.shape == (0,):
      if self.no_face_counter >= self.frame_patience:
        self.face_coords.data = self.NO_FACE
        self._coords_pub.publish(self.face_coords)
      else: # Do nothing (latch last coords published) 
        self.no_face_counter += 1

    else: # Get the face with the biggest bounding box
      if faces.shape == (5,):
        face_close = faces[0]
      else:
        best_conf = np.argmax(np.abs((faces[:, 0]-faces[:, 2]) * (faces[:,1]-faces[:,3])))
        face_close = faces[best_conf]

      x = face_close[0]
      y = face_close[1]
      w = face_close[2]
      h = face_close[3]
      cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

      # print('shape ', cv_image.shape)
      rospy.loginfo("Detected face: x=%i  y=%i", x+w, y+h)

      self.face_coords.data = [x+w, y+h]
      self._coords_pub.publish(self.face_coords)
      self.no_face_counter = 0

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = imageConverter()
  rospy.init_node('face_detector_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)