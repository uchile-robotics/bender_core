#!/usr/bin/python

__author__ = 'gdiaz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Bool, Int16, String
from beginner_tutorials.msg import LedCommand
from beginner_tutorials.msg import ServoCommand

# Use HW interface
from head_hw_v2 import HeadHW, SERVO0, SERVO1, SERVO2, SERVO3, SERVO4, SERVO_SELECT_STATE, SERVO_POS_STATE, LED_SELECT_STATE, LED_COLOR_STATE
#from dynamixel_driver.dynamixel_io import DynamixelIO
from dynamixel_io import DynamixelIO

DEV_ID = 0

class HeadController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.head = HeadHW(dxl_io, dev_id = DEV_ID)
        #self.servo_select_state = String()
        #self.servo_pos_state = Int16()
        #self.led_select_state = String()
        #self.led_color_state = String()
        self.servo_state = ServoCommand()
        self.led_state = LedCommand()
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        #self.servo_select_state.data = 0
        #self.servo_pos_state.data = 0
        #self.led_select_state.data = 0
        #self.led_color_state.data = 0
        self.servo_state.select = "Servo2"
        self.servo_state.pos = 0
        self.led_state.led = "led0"
        self.led_state.color = "1"

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
		#subscribers
        #self.command_servo_select_sub = rospy.Subscriber(self.controller_namespace + '/command_servo_select', String, self.process_servo_select_command)
        #self.command_pos_sub = rospy.Subscriber(self.controller_namespace + '/command_pos', Int16, self.process_pos_command)
        #self.led_select_sub = rospy.Subscriber(self.controller_namespace + '/led_select', String, self.process_led_select)
        #self.led_color_sub = rospy.Subscriber(self.controller_namespace + '/led_color', String, self.process_led_color)
        self.servo_sub = rospy.Subscriber(self.controller_namespace + '/servo_command', ServoCommand, self.process_servo_command)
        self.led_sub = rospy.Subscriber(self.controller_namespace + '/led_command', LedCommand, self.process_led_command)
       #publishers
        #self.state_servo_select_pub = rospy.Publisher(self.controller_namespace + '/servo_select_state', String)
        #self.state_servo_pos_pub = rospy.Publisher(self.controller_namespace + '/pos_state', Int16)
        #self.state_led_select_pub = rospy.Publisher(self.controller_namespace + '/led_select_state', String)
        #self.state_led_color_pub = rospy.Publisher(self.controller_namespace + '/led_color_state', String)
        self.state_servo_pub = rospy.Publisher(self.controller_namespace + '/servo_state', ServoCommand)
        self.state_led_pub = rospy.Publisher(self.controller_namespace + '/led_state', LedCommand)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_servo_command(self, msg):
        if (msg.select == "swapAll"):
            self.head.parallelSwapServos()
        elif (msg.select == "swapN"):
            self.head.swapServo(msg.pos)
        else:
            servo_selected = int(msg.select[5:])
            self.head.moveServoTo(servo_selected, msg.pos)

    def process_led_command(self, msg):
        if (msg.led == "surprised"):
            self.head.sendSurprised()
        elif (msg.led == "angry"):
            self.head.sendAngry()
        elif (msg.led == "reset"):
            self.head.sendResetColors()
        else:
            numLed = int(msg.led[3:])
            color = int(msg.color)
            self.head.changeLedColor(numLed, color)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            #get current states
            current_servo_select_state = self.head.get_state(SERVO_SELECT_STATE)
            current_servo_pos_state = self.head.get_state(SERVO_POS_STATE)
            current_led_select_state = self.head.get_state(LED_SELECT_STATE)
            current_led_color_state = self.head.get_state(LED_COLOR_STATE)

            #update the data state
            if current_servo_select_state == 0:
                self.servo_state.select = "Servo0"
            elif current_servo_select_state == 1:
                self.servo_state.select = "Servo1"
            elif current_servo_select_state == 2:
                self.servo_state.select = "Servo2"
            elif current_servo_select_state == 3:
                self.servo_state.select = "Servo3"
            elif current_servo_select_state == 4:
                self.servo_state.select = "Servo4"
            elif current_servo_select_state == 5:
                pass
            self.servo_state.pos = current_servo_pos_state
            if (current_led_select_state == 0xFC):
                pass
            else:
                self.led_state.led = "led"+str(current_led_select_state)
            self.led_state.color = str(current_led_color_state)

            #publish the state
            #self.state_servo_select_pub.publish(self.servo_select_state)
            #self.state_servo_pos_pub.publish(self.servo_pos_state)
            self.state_servo_pub.publish(self.servo_state)
            self.state_led_pub.publish(self.led_state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('head_controller')
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
    head = HeadController(dxl, 'head', 'left')
    head.initialize()
    head.start()
    rospy.spin()
    head.stop()
