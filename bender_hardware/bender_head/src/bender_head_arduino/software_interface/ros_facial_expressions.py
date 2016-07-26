#!/usr/bin/python

__author__ = 'gdiaz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Bool, Int16, String
from bender_head_arduino.msg import ExpressionCommand

# Use HW interface
from head_hw_controller import HeadHWController
from servos_hw import ServosHW
from eye_emotions import EyeEmotion
from facial_gestures import FacialGestures
from facial_expressions import FacialExpressions
#from dynamixel_driver.dynamixel_io import DynamixelIO
from dynamixel_io import DynamixelIO

DEV_ID = 0

class ROSFacialExpressions:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.hw_controller = HeadHWController(dxl_io, dev_id = DEV_ID)
        self.servos_hw = ServosHW(self.hw_controller)
        self.eyes = EyeEmotion(self.hw_controller)
        self.facial_gestures = FacialGestures(self.servos_hw)
        self.facial_expressions = FacialExpressions(self.eyes, self.facial_gestures)
        self.expression_state = ExpressionCommand()
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        self.expression_state.expression = "defaulf"

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
		#subscribers
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/expression_command', ExpressionCommand, self.process_command)
       #publishers
        self.state_pub = rospy.Publisher(self.controller_namespace + '/expression_state', ExpressionCommand)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_command(self, msg):
        if (msg.expressions == "surprised"):
            self.facial_expressions.surprised()
        elif (msg.expressions == "angry"):
            self.facial_expressions.angry()
        elif (msg.expressions == "happy"):
            self.facial_expressions.happy()
        elif (msg.expressions == "sad"):
            self.facial_expressions.angry()
        elif (msg.expressions == "default"):
            self.facial_expressions.default()

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            current_expression = self.facial_expressions.get_state()

            #update the data state
            self.expression_state.expression = current_expression

            #publish the state
            self.state_pub.publish(self.expression_state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('expressions_controller')
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
    expressions = ROSFacialExpressions(dxl, 'expressions', 'left')
    expressions.initialize()
    expressions.start()
    rospy.spin()
    expressions.stop()
