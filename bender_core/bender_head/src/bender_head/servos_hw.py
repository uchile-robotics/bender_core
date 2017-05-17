#!/usr/bin/python

import roslib
import time
from dynamixel_driver.dynamixel_io import DynamixelIO

roslib.load_manifest('bender_head')

# NON ROS HARDWARE INTERFACE

"""The ServosHW class allows to actuate the organs (servos) by a percentage, without the need to know the physical restrictions.
It use the intermediate level methods defined in HeadHW.
IMPORTAT: Modify only if you are sure of physical restrictions of the actuators"""

# Use HW controller
from head_hw import HeadHW, SERVO0_POS, SERVO1_POS, SERVO2_POS, SERVO3_POS, SERVO4_POS, SERVO5_POS

class ServosHW(object):
	def __init__(self, hw_controller):
		self.hw_controller = hw_controller

	def remapRange(self, value, toLow, toHigh):
		if (value<0 or value>100):
			return "Warning value out of range"
		fromLow = 0
		fromHigh = 100
		new_range = toHigh-toLow
		old_range = fromHigh - fromLow
		new_value = (float(new_range)/old_range)*(value-fromLow)+toLow
		return int(new_value)

	def left_ear(self, lifting_percentage):
		if (lifting_percentage != -1):
			angle = self.remapRange(lifting_percentage, 160, 50)
			self.hw_controller.moveServoTo(SERVO0_POS, angle)

	def right_ear(self, lifting_percentage):
		if (lifting_percentage != -1):
			angle = self.remapRange(lifting_percentage, 110, 40)
			self.hw_controller.moveServoTo(SERVO3_POS, angle)
		
	def left_eyebrow(self, lifting_percentage):
		if (lifting_percentage != -1):
			angle = self.remapRange(lifting_percentage, 140, 40)
			self.hw_controller.moveServoTo(SERVO2_POS, angle)
		
	def right_eyebrow(self, lifting_percentage):
		if (lifting_percentage != -1):
			angle = self.remapRange(lifting_percentage, 40, 140)
			self.hw_controller.moveServoTo(SERVO4_POS, angle)
		
	def mouth(self, opening_percentage):
		if (opening_percentage != -1):
			angle = self.remapRange(opening_percentage, 90, 140)
			self.hw_controller.moveServoTo(SERVO1_POS, angle)

	def neck(self, angle):
		#if (opening_percentage != -1):
			#angle = self.remapRange(opening_percentage, 60, 120)
		angle2 = 90-angle
		self.hw_controller.moveServoTo(SERVO5_POS, int(angle2*0.8))

if __name__ == '__main__':
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	hw_controller = HeadHW(dxl, dev_id = DEV_ID)
	servos = ServosHW(hw_controller)
	while True:
		servos.left_ear(50)
		servos.right_ear(50)
		servos.left_eyebrow(50)
		servos.right_eyebrow(50)
		servos.mouth(50)
