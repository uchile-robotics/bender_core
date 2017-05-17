#!/usr/bin/python

__author__ = 'gdiaz'

import rospy
import random

from threading import Thread

from std_msgs.msg import Bool, UInt8
from uchile_msgs.msg import FaceEmotion

# Use HW interface
from bender_head.head_hw import HeadHW
from bender_head.servos_hw import ServosHW
from bender_head.emotions_manager import EmotionsManager

class FaceController(object):

    DEV_ID = 16
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        """
        Provides a high level interface over ROS to generate emotions on Bender Robot.
        It use methods provided by FacialExpressions class (non ROS hardware interface). See Documentation.
        """
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        print dxl_io
        for i in range(10):
            print dxl_io.ping(35)
            rospy.sleep(0.01)

        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace

        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        rospy.logwarn("Using rate: {}".format(self.state_update_rate))
        self.device_id = rospy.get_param(self.controller_namespace + '/id', 1)
        rospy.logwarn("Using id: {}".format(self.device_id))
        self.head_interface = HeadHW(dxl_io, dev_id=self.device_id)
        self.servos_hw = ServosHW(self.head_interface)
        self.emotions_controller = EmotionsManager(self.head_interface, self.servos_hw)
        self.emotion_state = FaceEmotion()
    
        # valid emotions        
        self.static_emotion_list = self.emotions_controller.emotions
        self.dynamic_emotion_list = self.emotions_controller.dynamic_emotions
        
    def initialize(self):
        self.emotion_state.emotion_name = "default"
        return True

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        # subscribers
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/emotion_cmd', FaceEmotion, self.process_command)
        self.move_mouth_sub = rospy.Subscriber(self.controller_namespace + '/move_mouth', Bool, self.move_mouth)
        self.brightness_sub = rospy.Subscriber(self.controller_namespace + '/brightness', UInt8, self.set_brightness)

        # publishers
        self.state_pub = rospy.Publisher(self.controller_namespace + '/emotion_state', FaceEmotion, queue_size=50)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def set_brightness(self, msg):
        self.head_interface.set_brightness(msg.data)

    def process_command(self, msg):
        if msg.emotion_name in self.static_emotion_list:
            self.emotions_controller.set_emotion(msg.emotion_name)
        elif msg.emotion_name in self.dynamic_emotion_list:
            self.emotions_controller.set_dynamic_emotion(msg.emotion_name)
        else:
            rospy.logwarn("Unknown emotion_name: '" + msg.emotion_name)

    def move_mouth(self, msg):
        if msg.data:
            if random.random() < 0.5:
                self.emotions_controller.set_dynamic_emotion('talk1')
            else:
                self.emotions_controller.set_dynamic_emotion('talk2')

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            current_expression = self.emotions_controller.get_state()

            #update the data state
            self.emotion_state.emotion_name = current_expression

            #publish the state
            self.state_pub.publish(self.emotion_state)
            rate.sleep()
