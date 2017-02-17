#!/usr/bin/python

__author__ = 'gdiaz'


# ROS INTERFACE

"""Test for emotions on Bender Robot."""

import rospy
import time
    
from bender_msgs.msg import FaceEmotion

class EmotionTest(object):

    def __init__(self):
        # Only argument stuff
        self.running = False
        self.state_update_rate = 5
        self.msg = FaceEmotion()
        self.emotions = rospy.get_param('/bender/emotions')
        self.dynamic_emotions = rospy.get_param('/bender/dynamic_emotions')

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #subscribers
        #publishers
        self.emotion_pub = rospy.Publisher('/bender/led_head_controller/emotion_cmd', FaceEmotion, queue_size = 50)
        # Thread(target=self.run_test).start()

    def run_static(self):
        # rate = rospy.Rate(self.state_update_rate)
        # while self.running and not rospy.is_shutdown():
        rospy.logwarn("Starting Static Emotions Test")
        for emotion, values in self.emotions.iteritems():
            rospy.loginfo("Emotion: {}".format(emotion))
            self.msg.emotion_name = emotion
            self.emotion_pub.publish(self.msg)
            time.sleep(2)

    def run_dynamic(self):
        rospy.logwarn("Starting Dynamic Emotions Test")
        for emotion, values in self.dynamic_emotions.iteritems():
            rospy.loginfo("Emotion: {}".format(emotion))
            self.msg.emotion_name = emotion
            self.emotion_pub.publish(self.msg)
            time.sleep(5)
            # rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Test_Emotions')
    emotions_test = EmotionTest()
    emotions_test.start()
    emotions_test.run_static()
    emotions_test.run_dynamic()
