#!/usr/bin/python

import roslib; roslib.load_manifest('bender_head')
import time

# NON ROS HARDWARE INTERFACE

"""The FacialGestures class define the position of each servo for an specific emotion defined as a method of the class.
It does so using the methods of ServosHW class, which receive percentages to move the actuators (servos).
Modify to create new emotions based on actuators states."""

"""
Method Parameters:
lifting_percentage: Valid for methods 'left_ear, right_ear, left_eyebrow, right_eyebrow'. A number between 0 and 100.
opening_percentage: Valid for method 'mouth'
"""

# Use HW controllers
from head_hw_controller import HeadHWController
from servos_hw import ServosHW

class FacialGestures(object):
	def __init__(self, servos_hw):
		self.servos_hw = servos_hw

	def surprised(self):
		self.servos_hw.left_ear(100)
		self.servos_hw.right_ear(100)
		self.servos_hw.left_eyebrow(82)
		self.servos_hw.right_eyebrow(55)
		self.servos_hw.mouth(90)

	def angry(self):
		self.servos_hw.left_ear(40)
		self.servos_hw.right_ear(40)
		self.servos_hw.left_eyebrow(0)
		self.servos_hw.right_eyebrow(0)
		self.servos_hw.mouth(0)

	def happy(self):
		self.servos_hw.left_ear(70)
		self.servos_hw.right_ear(70)
		self.servos_hw.left_eyebrow(70)
		self.servos_hw.right_eyebrow(65)
		self.servos_hw.mouth(50)

	def sad(self):
		self.servos_hw.left_ear(10)
		self.servos_hw.right_ear(10)
		self.servos_hw.left_eyebrow(90)
		self.servos_hw.right_eyebrow(90)
		self.servos_hw.mouth(0)
		
	def veryHappy(self):		
		self.servos_hw.left_eyebrow(70)
		self.servos_hw.right_eyebrow(70)
		self.servos_hw.mouth(20)
		self.servos_hw.left_ear(40)
		self.servos_hw.right_ear(40)
		time.sleep(0.2)
		self.servos_hw.left_ear(20)
		self.servos_hw.right_ear(20)
		time.sleep(0.2)
		self.servos_hw.left_ear(60)
		self.servos_hw.right_ear(60)
		time.sleep(0.2)
		self.servos_hw.left_ear(30)
		self.servos_hw.right_ear(30)
		time.sleep(0.2)
		self.servos_hw.left_ear(50)
		self.servos_hw.right_ear(50)
		time.sleep(0.2)
		self.servos_hw.left_ear(40)
		self.servos_hw.right_ear(40)
		
	def default(self):
		self.servos_hw.left_ear(100)
		self.servos_hw.right_ear(100)
		self.servos_hw.left_eyebrow(50)
		self.servos_hw.right_eyebrow(50)
		self.servos_hw.mouth(10)


if __name__ == '__main__':
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	hw_controller = HeadHWController(dxl, dev_id = DEV_ID)
	servos_hw = ServosHW(hw_controller)
	facial_gestures = FacialGestures(servos_hw)
	while True:
		facial_gestures.surprised()
		time.sleep(2)
		facial_gestures.angry()
		time.sleep(2)
		facial_gestures.happy()
		time.sleep(2)
		facial_gestures.sad()
		time.sleep(2)
		facial_gestures.default()
		time.sleep(2)
