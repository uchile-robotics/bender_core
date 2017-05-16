#!/usr/bin/env python

import rospy
from uchile_fun.srv import UploadImage, UploadImageRequest, UploadImageResponse
import InstagramAPI

class InstagramROS(object):
    def __init__(self):
        rospy.Service('photo_upload', UploadImage, self.upload_image)
        user = rospy.get_param('~user')
        password = rospy.get_param('~password')
        path = rospy.get_param('~instagram_path')
        self.instagram = InstagramAPI.Instagram(user, password, IGDataPath=path)
        try:
            self.instagram.login()
        except Exception as e:
            e.message
            exit()
        
    def upload_image(self, req):
        resp = UploadImageResponse()
        rospy.loginfo("Uploading image {}".format(req.path))
        try:
            self.instagram.uploadPhoto(req.path, req.caption)
            resp.success = True
        except Exception as e:
            resp.success = False
            rospy.logerr(e)
        return resp

    def logout(self):
        self.instagram.logout()

if __name__ == '__main__':
    rospy.init_node('instagram_node')
    rospy.loginfo('Instagram ROS')
    try:
        insta = InstagramROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo('Closing Instagram')
