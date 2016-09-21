#!/usr/bin/python

import roslib; roslib.load_manifest('bender_head')
import time
from dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

"""The ServosHW class allows to actuate the organs (servos) by a percentage, without the need to know the physical restrictions.
It use the intermediate level methods defined in HeadHWController class.
IMPORTAT: Modify only if you are sure of physical restrictions of the actuators"""

# Use HW controller
from head_hw_controller import HeadHWController, SERVO0, SERVO1, SERVO2, SERVO3, SERVO4

class ServosHW(object):
	def __init__(self, hw_controller):
		self.hw_controller = hw_controller

	def remapRange(self, value, toLow, toHigh):
		if (value<0 or value>100): return "Warning value out of range"
		fromLow = 0
		fromHigh = 100
		new_range = toHigh-toLow
		old_range = fromHigh - fromLow
		new_value = (float(new_range)/old_range)*(value-fromLow)+toLow
		return int(new_value)

	def left_ear(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 180, 50)
		self.hw_controller.moveServoTo(SERVO0, angle)

	def right_ear(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 130, 0)
		self.hw_controller.moveServoTo(SERVO3, angle)
		
	def left_eyebrow(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 40, 140)
		self.hw_controller.moveServoTo(SERVO2, angle)
		
	def right_eyebrow(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 140, 40)
		self.hw_controller.moveServoTo(SERVO4, angle)
		
	def mouth(self, opening_percentage):
		angle = self.remapRange(opening_percentage, 110, 140)
		self.hw_controller.moveServoTo(SERVO1, angle)

if __name__ == '__main__':
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	hw_controller = HeadHWController(dxl, dev_id = DEV_ID)
	servos = ServosHW(hw_controller)
	while True:
		servos.left_ear(50)
		servos.right_ear(50)
		servos.left_eyebrow(50)
		servos.right_eyebrow(50)
		servos.mouth(50)
