#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

from bender_core.robot_skill import RobotSkill
from sensor_msgs.msg import Image


class CameraSkill(RobotSkill):

    _type = "camera"

    def __init__(self):
        super(CameraSkill, self).__init__()
        self._description = "Camera node skill"
        self._cam_topic = "/bender/sensors/selfiecam/image_raw"
        self.bridge = CvBridge()

    def check(self):
        self.logdebug("Sound service [OK]")
        return True
    
    def setup(self):
        return True

    def shutdown(self):
        return True

    def start(self):
        return True

    def pause(self):
        return True

    def get_last_image(self, timeout=1):
        try:
            return rospy.wait_for_message(self._cam_topic, Image, timeout)
        except rospy.ROSException:
            return None

    def imgmsg_to_cv2(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return None

    def cvimg_to_file(self, cv_image, filename):
        try:
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Successfully saved image to {}".format(filename))
            return True
        except:
            rospy.logwarn("Failed to save image to {}".format(filename))
            return False

    def imgmsg_to_file(self, msg, filename):
        image = self.imgmsg_to_cv2(msg)
        if image is None:
            rospy.logwarn("Failed to convert msg to cv2 image")
            return False
        return self.cvimg_to_file(image, filename)

    def apply_effect_cartoon(self, cv_image):

        num_down = 2  # number of downsampling steps
        num_bilateral = 7  # number of bilateral filtering steps

        # downsample image using Gaussian pyramid
        img_color = cv_image
        for _ in xrange(num_down):
            img_color = cv2.pyrDown(img_color)

        # repeatedly apply small bilateral filter instead of
        # applying one large filter
        for _ in xrange(num_bilateral):
            img_color = cv2.bilateralFilter(img_color, d=9,
                                            sigmaColor=9,
                                            sigmaSpace=7)

        # upsample image to original size
        for _ in xrange(num_down):
            img_color = cv2.pyrUp(img_color)

        # convert to grayscale and apply median blur
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        img_blur = cv2.medianBlur(img_gray, 7)

        # detect and enhance edges
        img_edge = cv2.adaptiveThreshold(img_blur, 255,
                                         cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY,
                                         blockSize=9,
                                         C=2)

        # convert back to color, bit-AND with color image
        img_edge = cv2.cvtColor(img_edge, cv2.COLOR_GRAY2RGB)
        img_cartoon = cv2.bitwise_and(img_color, img_edge)

        return img_cartoon

    def save_last_img(self, filename):
        msgimage = self.get_last_image()
        if msgimage is None:
            return False
        cv_image = self.imgmsg_to_cv2(msgimage)
        if cv_image is None:
            return False
        self.cvimg_to_file(cv_image, filename)

    def display(self, cv_image, timeout=0):
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(timeout)
        return cv_image

    def get_path(self, filename):
        return os.path.abspath(filename)

