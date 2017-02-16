#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""
Provides a high level interface over ROS to generate emotions on Bender Robot.
It use methods provided by FacialExpressions class (non ROS hardware interface). See Documentation.
"""
import sys
import math
import rospy
import random
import time

from threading import Thread

from std_msgs.msg import Empty,Bool
from bender_msgs.msg import ExpressionCommand
from bender_msgs.msg import Emotion

# Use HW interface
from head_hw import HeadHW
from servos_hw import ServosHW
from emotions_manager import EmotionsManager
from dynamixel_driver.dynamixel_io import DynamixelIO


class LedHeadController(object):

    DEV_ID = 16
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace

        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        rospy.logwarn("Using rate: {}".format(self.state_update_rate))
        self.device_id = rospy.get_param(self.controller_namespace + '/id', 1)
        rospy.logwarn("Using id: {}".format(self.device_id))
        self.head_interface = HeadHW(dxl_io, dev_id = self.device_id)
        self.servos_hw = ServosHW(self.head_interface)
        self.emotions_controller = EmotionsManager(self.head_interface, self.servos_hw)
        self.expression_state = ExpressionCommand()
        self.joystick_msg = ExpressionCommand()
        # valid emotions        
        #self.static_emotion_list = [ 'happy' , 'sad' , 'angry' , 'surprised' , 'apagado']
        self.static_emotion_list = self.emotions_controller.emotions
        self.dynamic_emotion_list = self.emotions_controller.dynamic_emotions
        
    def initialize(self):
        # Get params and allocate msgs
        # self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        # rospy.logerr("Using rate: {}".format(self.state_update_rate))
        # self.device_id = rospy.get_param(self.controller_namespace + '/id', 1)
        # rospy.logerr("Using id: {}".format(self.device_id))
        self.expression_state.expression = "defaulf"
        return True

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #subscribers
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/emotion_command', ExpressionCommand, self.process_command)
        self.expressionsList_sub = rospy.Subscriber(self.controller_namespace + '/emotion_list', Empty, self.list_expressions)
        self.joystick_sub = rospy.Subscriber(self.controller_namespace + '/cmd', Emotion, self.joystick_cmd)
        self.move_mouth_sub = rospy.Subscriber(self.controller_namespace + '/move_mouth', Bool, self.move_mouth)
       #publishers
        self.state_pub = rospy.Publisher(self.controller_namespace + '/emotion_state', ExpressionCommand, queue_size = 50)
        self.joy_pub = rospy.Publisher(self.controller_namespace +'/emotion_command', ExpressionCommand, queue_size = 50)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.expressionsList_sub.unregister()
        self.joystick_sub.unregister()
        self.state_pub.unregister()
        self.joy_pub.unregister()

    def process_command(self, msg):
        if (msg.expression in self.static_emotion_list):
            self.emotions_controller.set_emotion(msg.expression)
        elif (msg.expression in self.dynamic_emotion_list):
            self.emotions_controller.set_dynamic_emotion(msg.expression)
        elif (msg.expression == "id"):
            device_id = self.head_interface.get_state(3)
            rospy.logwarn("device_id = "+str(device_id))
        elif (msg.expression[0] == "-"):
            self.emotions_controller.moveNeck(int(msg.expression[1:]))
        else:
            rospy.logwarn("For order 'changeFace', unknown action: '" + msg.expression + "' ... Please use one of the following:\n" + str(self.static_emotion_list))

    def joystick_cmd(self, msg):
        if (msg.Order == "changeFace"):
            self.joystick_msg.expression = msg.Action
            self.joy_pub.publish(self.joystick_msg)
        elif (msg.Order == "MoveX"):
            print('STDMSG: Mover servo a:'+str(msg.X))
            self.joystick_msg.expression = '-'+str(msg.X)
            self.joy_pub.publish(self.joystick_msg)
        elif (msg.Order == "changeID"):
            try: 
                new_id = int(msg.X)
                self.head_interface.change_id(new_id)
                device_id = self.head_interface.get_state(3)
                rospy.logwarn("New device_id = "+str(device_id))
                self.device_id = device_id
            except:
                rospy.logerr("Invalid ID: {}, should be an Integer".format(msg.X))


    def list_expressions(self, msg):
        print("------------------------------------------------------------")
        print("Expressions Availables:\n")
        print("surprised\nangry\nhappy\nsad\nveryHappy\ndefault\napagado\n")
        print("------------------------------------------------------------")

    def move_mouth(self,msg):
        if random.random() < 0.5:
            self.joystick_msg.expression = 'talk1'
        else:
            self.joystick_msg.expression = 'talk2' 

        if msg.data is True:
            self.joy_pub.publish(self.joystick_msg)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            current_expression = self.emotions_controller.get_state()

            #update the data state
            self.expression_state.expression = current_expression

            #publish the state
            self.state_pub.publish(self.expression_state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('expressions_controller')
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate = 200000)
    # for i in range(50):
    #     print dxl.ping(16)
    #     time.sleep(0.01)
    expressions = LedHeadController(dxl, 'emotions', 'left')
    expressions.initialize()
    expressions.start()    
    rospy.spin()
    expressions.stop()