#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS to generate emotions on Bender Robot.
It use methods provided by FacialExpressions class (non ROS hardware interface). See Documentation.
"""
import sys
import math
import rospy
import random

from threading import Thread

from std_msgs.msg import Empty,Bool
from bender_msgs.msg import ExpressionCommand
from bender_msgs.msg import Emotion

# Use HW interface
from head_hw_controller import HeadHWController
from servos_hw import ServosHW
from emotion_controller import EmotionsController
#from dynamixel_driver.dynamixel_io import DynamixelIO
from dynamixel_io import DynamixelIO

DEV_ID = 16

class HeadController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.hw_controller = HeadHWController(dxl_io, dev_id = DEV_ID)
        self.servos_hw = ServosHW(self.hw_controller)
        self.emotions_controller = EmotionsController(self.hw_controller, self.servos_hw)
        self.expression_state = ExpressionCommand()
        self.joystick_msg = ExpressionCommand()
        # valid emotions        
        #self.static_emotion_list = [ 'happy' , 'sad' , 'angry' , 'surprised' , 'apagado']
        self.static_emotion_list = self.emotions_controller.emotions
        self.dynamic_emotion_list = self.emotions_controller.dynamic_emotions
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        self.expression_state.expression = "defaulf"

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #subscribers
        self.command_sub = rospy.Subscriber('~emotion_command', ExpressionCommand, self.process_command)
        self.expressionsList_sub = rospy.Subscriber('~emotion_list', Empty, self.list_expressions)
        self.joystick_sub = rospy.Subscriber('~cmd', Emotion, self.joystick_cmd)
        self.move_mouth_sub = rospy.Subscriber('/bender/hw/head/move_mouth', Bool, self.move_mouth)
       #publishers
        self.state_pub = rospy.Publisher('~emotion_state', ExpressionCommand, queue_size = 50)
        self.joy_pub = rospy.Publisher('~emotion_command', ExpressionCommand, queue_size = 50)
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
            device_id = self.hw_controller.get_state(3)
            print("device_id = "+str(device_id))
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
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate = 115200)
    expressions = HeadController(dxl, 'emotions', 'left')
    expressions.initialize()
    expressions.start()
    rospy.spin()
    expressions.stop()