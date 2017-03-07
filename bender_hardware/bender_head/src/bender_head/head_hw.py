#!/usr/bin/python

import sys
import time
import rospy
from dynamixel_driver.dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

"""The HeadHW class provides low-level methods to control the 'Servos & LEDs arduino device'
using Dynamixel protocol, provided by DynamixelIO class.
IMPORTANT: Modify only if you are sure of hardware specifications. See Documentation"""

# MMap position for commands (mem Addrs)

SERVO0_POS = 6
SERVO1_POS = 7 
SERVO2_POS = 8 
SERVO3_POS = 9 
SERVO4_POS = 10
SERVO5_POS = 11
SERVO_CMD = 12
LED_SELECT = 13
LED_COLOR = 14
LED_BRIGHTNESS = 15
LED_CMD = 16

# Servos commands (not addr)
SERVO1_SWAP = 20
SERVOS_INACTIVE = 21

# LEDs commands (not addr)
SHOW_R1 = 1
SHOW_R2 = 2
UPDATE_C = 3
CHANGE_BRIGHT = 4
SHOW_RAINBOW = 5
LEDS_INACTIVE = 21

class HeadHW(object):
    #red = 
    def __init__(self, dxl_io, dev_id = 1):
        self.dxl = dxl_io
        self.id = dev_id
        self.state = [0]

    def write_addr(self, addr, value):
        result = []
        try:
            result = self.dxl.write(self.id, addr, [value])
        except Exception as e:
            rospy.logwarn('Exception thrown while writing addres %d, value %d' % (addr, value))
        return result

    def ping(self):
        result = []
        try:
            result = self.dxl.ping(self.id)
        except Exception as e:
            rospy.logwarn('Exception thrown while pinging device %d - %s' % (self.id, e))
        return result

    def get_state(self, state_variable):
        result = []
        try:
            result = self.dxl.read(self.id, state_variable, 1)
        except Exception as e:
            rospy.logwarn('Exception thrown while writing addres %d' % (state_variable))
            # return e
            self.state = result[5]
        return self.state

    def change_id(self, new_id):
        if new_id<32:
            print 'Not Allow ID:%d. Must be greater than 31' % (new_id)
            return
        self.write_addr(3, new_id)

    def set_brightness(self, bright):
        if (bright < 0) or (bright > 255):
            print 'brightness %d out of range' % (bright)
            return
        self.write_addr(LED_BRIGHTNESS, bright)
        self.write_addr(LED_CMD, CHANGE_BRIGHT)

    def show_rainbow(self):
        self.write_addr(LED_CMD, SHOW_RAINBOW)
        time.sleep(1)

    def moveServoTo(self, servo_addr, pos):
        self.write_addr(servo_addr, pos)            #Update servo position
        self.write_addr(SERVO_CMD, servo_addr)      #Move Servo (call to servo.write(pos) in Arduino)

    def swapServo(self, servo_addr):
        for i in range(180):
            self.moveServoTo(servo_addr, i)
        for i in range(180,0,-1):
            self.moveServoTo(servo_addr, i)

    def parallelSwapServos(self):
        for pos in range(0,180,10):
            for servo_addr in [SERVO0_POS, SERVO1_POS, SERVO2_POS, SERVO3_POS, SERVO4_POS, SERVO5_POS]:
                self.moveServoTo(servo_addr, pos)

        for pos in range(180,0,-10):
            for servo_addr in [SERVO0_POS, SERVO1_POS, SERVO2_POS, SERVO3_POS, SERVO4_POS, SERVO5_POS]:
                self.moveServoTo(servo_addr, pos)

    def updateLedColor(self, numLed, r_color, g_color, b_color):
        if (numLed < 0) or (numLed > 40):
            rospy.logwarn('LED %d out of range' % (numLed))
            return
        encode8bitColor = ((r_color * 7 / 255) << 5) + ((g_color * 7 / 255) << 2) + ((b_color * 3 / 255))
        self.write_addr(LED_SELECT, numLed)
        self.write_addr(LED_COLOR, encode8bitColor)
        self.write_addr(LED_CMD, UPDATE_C)
        self.write_addr(LED_CMD, LEDS_INACTIVE)

    def showColors(self, eye):
        if (eye == "left"):
            self.write_addr(LED_CMD, SHOW_R1)           #This command call "show" method for each led in left eye on Arduino
        elif (eye == "right"):
            self.write_addr(LED_CMD, SHOW_R2)       #This command call "show" method for each led in right eye on Arduino
        time.sleep(0.1)

    def changeLedColor(self, numLed, rgb_color):
        self.updateLedColor(numLed, rgb_color[0], rgb_color[1], rgb_color[2])
        if numLed>=0 and numLed<21:
            self.showColors("left")
        elif numLed>20 and numLed<40:
            self.showColors("right")

    def set_eye_colors(self, eye, rgb_colors):
        n_leds = 20
        if (eye!="left" and eye!="right"):
            print "parameter eye must be <left> or <right>, <%s> given" %(eye)
            return
        if len(rgb_colors)!=n_leds:
            print "bad number of colors: %d. Must be 20" %(len(rgb_colors))
            return
        if (eye=="left"):
            #print "left"
            for i_led in range(0,n_leds,1):
                color = rgb_colors[i_led]
                r_color = color[0]
                g_color = color[1]
                b_color = color[2]
                self.updateLedColor(i_led, r_color, g_color, b_color)
            self.showColors(eye)
        elif (eye=="right"):
            #print "right"
            """
            Map:
            20 -> 15
            21 -> 14
            22 -> 13
            ...
            35 -> 0

            36 -> 16
            37 -> 17
            38 -> 18
            39 -> 19
            """
            for i_led in range(n_leds,2*n_leds-4,1): #i:20->35
                color = rgb_colors[2*n_leds - i_led - 5]
                r_color = color[0]
                g_color = color[1]
                b_color = color[2]
                self.updateLedColor(i_led, r_color, g_color, b_color)
            for i_led in range(2*n_leds-4,2*n_leds,1): #i:36->39
                color = rgb_colors[i_led - 20]
                r_color = color[0]
                g_color = color[1]
                b_color = color[2]
                self.updateLedColor(i_led, r_color, g_color, b_color)
            self.showColors(eye)

if __name__ == '__main__':
    DEV_ID = 35
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 200000)
    head = HeadHW(dxl, dev_id = DEV_ID)
    red = [100,0,0]
    green = [0,100,0]
    blue = [0,0,100]
    while True:
        head.changeLedColor(0, green)
        print head.ping()
        head.moveServoTo(SERVO2_POS, 0)
        head.changeLedColor(0, blue)
        for pos in range(20, 160, 10):
            head.moveServoTo(SERVO2_POS, pos)
        head.changeLedColor(0, red)
        # head.parallelSwapServos()