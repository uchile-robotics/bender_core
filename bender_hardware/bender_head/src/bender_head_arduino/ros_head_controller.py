#!/usr/bin/python

__author__ = 'gdiaz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Bool, Int16, String
from bender_head_arduino.msg import LedCommand2
from bender_head_arduino.msg import ServoCommand
from bender_head_arduino.msg import ServoState

# Use HW interface
from head_hw import HeadHW, SERVO0, SERVO1, SERVO2, SERVO3, SERVO4, SERVO_SELECT_STATE, SERVO_POS_STATE, LED_SELECT_STATE, LED_COLOR_STATE
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
        self.servos_state = ServoState()
        self.led_state = LedCommand2()
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        self.servos_state.servo0 = 0
        self.servos_state.servo1 = 1
        self.servos_state.servo2 = 2
        self.servos_state.servo3 = 3
        self.servos_state.servo4 = 4
        self.servos_state.lastCommand = "none"
        self.led_state.led = "led0"
        self.led_state.r_color = 0
        self.led_state.g_color = 0
        self.led_state.b_color = 0

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
		#subscribers
        self.servo_sub = rospy.Subscriber(self.controller_namespace + '/servo_command', ServoCommand, self.process_servo_command)
        self.led_sub = rospy.Subscriber(self.controller_namespace + '/led_command', LedCommand2, self.process_led_command)
       #publishers
        self.state_servo_pub = rospy.Publisher(self.controller_namespace + '/servos_state', ServoState)
        self.state_led_pub = rospy.Publisher(self.controller_namespace + '/led_state', LedCommand2)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_servo_command(self, msg):
        if (msg.select == "swapAll"):
            self.head.parallelSwapServos()
            self.servos_state.lastCommand = "swapAll"
        elif (msg.select == "swap"):
            self.head.swapServo(msg.pos)
            self.servos_state.lastCommand = "swap"
        else:
            #servo_selected = int(msg.select[5:])
            if (msg.select == "oreja izq"): 
                servo_selected = 0
                self.servos_state.servo0 = msg.pos
            elif (msg.select == "boca"): 
                servo_selected = 1
                self.servos_state.servo1 = msg.pos
            elif (msg.select == "ceja izq"): 
                servo_selected = 2
                self.servos_state.servo2 = msg.pos
            elif (msg.select == "oreja der"): 
                servo_selected = 3
                self.servos_state.servo3 = msg.pos
            elif (msg.select == "ceja der"): 
                servo_selected = 4
                self.servos_state.servo4 = msg.pos
            self.head.moveServoTo(servo_selected, msg.pos)
            self.servos_state.lastCommand = "servo "+str(servo_selected)+": simple movement"
    def process_led_command(self, msg):
        if (msg.led == "surprised"):
            self.head.sendSurprised()
            self.led_state.led = "surprised"
        elif (msg.led == "angry"):
            self.head.sendAngry()
            self.led_state.led = "angry"
        elif (msg.led == "palette1"):
            self.head.color_palette1()
            self.led_state.led = "palette1"
        elif (msg.led == "palette2"):
            self.head.color_palette2()
            self.led_state.led = "palette2"
        elif (msg.led == "reset"):
            self.head.sendResetColors()
            self.led_state.led = "reset"
        else:
            numLed = int(msg.led[3:])
            rgb_color = [msg.r_color, msg.g_color, msg.b_color]
            self.head.changeLedColor(numLed, rgb_color)
            self.led_state.led = "led"+msg.led[3:]

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            #get current states (reads from device memory)
            current_servo_select_state = self.head.get_state(SERVO_SELECT_STATE)
            current_servo_pos_state = self.head.get_state(SERVO_POS_STATE)
            current_led_select_state = self.head.get_state(LED_SELECT_STATE)
            current_led_color_state = self.head.get_state(LED_COLOR_STATE)

            #update the data state
            if current_servo_select_state[0] == 5:
                pass
            else:
                self.servos_state.lastCommand = "servo "+str(current_servo_select_state[0])+": simple movement"
                if (current_servo_select_state[0]==0): self.servos_state.servo0 = current_servo_pos_state[0]
                elif (current_servo_select_state[0]==1): self.servos_state.servo1 = current_servo_pos_state[0]
                elif (current_servo_select_state[0]==2): self.servos_state.servo2 = current_servo_pos_state[0]
                elif (current_servo_select_state[0]==3): self.servos_state.servo3 = current_servo_pos_state[0]
                elif (current_servo_select_state[0]==4): self.servos_state.servo4 = current_servo_pos_state[0]
            if (current_led_select_state[0] == 0xFC):
                pass
            else:
                self.led_state.led = "led"+str(current_led_select_state[0])
            self.led_state.r_color = current_led_color_state[0]
            self.led_state.g_color = current_led_color_state[1]
            self.led_state.b_color = current_led_color_state[2]

            #publish the state
            self.state_servo_pub.publish(self.servos_state)
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
