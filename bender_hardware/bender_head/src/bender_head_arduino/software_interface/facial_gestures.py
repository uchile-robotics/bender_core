#!/usr/bin/python

import time

# NON ROS HARDWARE INTERFACE

# Use HW controllers
from head_hw_controller import HeadHWController
from servos_hw import ServosHW

class FacialGestures(object, servos_hw):
	def __init__(self):
		self.servos_hw = servos_hw

	def surprised(self):
		self.servos_hw.left_ear(90)
		self.servos_hw.right_ear(90)
		self.servos_hw.left_eyebrow(60)
		self.servos_hw.right_eyebrow(70)
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
		self.servos_hw.right_eyebrow(70)
		self.servos_hw.mouth(20)

	def sad(self):
		self.servos_hw.left_ear(10)
		self.servos_hw.right_ear(10)
		self.servos_hw.left_eyebrow(90)
		self.servos_hw.right_eyebrow(90)
		self.servos_hw.mouth(0)
		
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
		sleep(2)
		facial_gestures.angry()
		sleep(2)
		facial_gestures.happy()
		sleep(2)
		facial_gestures.sad()
		sleep(2)
		facial_gestures.default()
		sleep(2)
