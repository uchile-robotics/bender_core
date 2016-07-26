#!/usr/bin/python

import time
from dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

# Use HW controller
from head_hw_controller import HeadHWController, SERVO0, SERVO1, SERVO2, SERVO3, SERVO4

class ServosHW(object, hw_controller):
	def __init__(self):
		self.hw_controller = hw_controller

	def remapRange(self, value, toLow, toHigh):
		if (value<0 or value>100): return "Warning value out of range"
		fromLow = 0
		fromHigh = 100
		new_range = toHigh-toLow
		old_range = fromHigh - fromLow
		new_value = (float(new_range)/old_range)*(value-fromLow)+toLow
		return new_value

	def left_ear(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 180, 50)
		self.hw_controller.moveServoTo(SERVO0, angle)

	def right_ear(self, lifting_percentage):
		angle = self.remapRange(lifting_percentage, 0, 130)
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
