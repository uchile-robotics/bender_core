#!/usr/bin/python

import time

# NON ROS HARDWARE INTERFACE

# Use HW controllers
from head_hw_controller import HeadHWController
from servos_hw import ServosHW
from eye_emotions import EyeEmotion
from facial_gestures import FacialGestures

class FacialExpressions(object, eye_emotions, facial_gestures):
	def __init__(self):
		self.eye_emotions = eye_emotions
		self.facial_gestures = facial_gestures
		self.actual_expresion = ""

	def surprised(self):
		self.eye_emotions.surprised()
		self.facial_gestures.surprised()

	def angry(self):
		self.eye_emotions.angry()
		self.facial_gestures.angry()

	def happy(self):
		self.eye_emotions.happy()
		self.facial_gestures.happy()

	def sad(self):
		self.eye_emotions.sad()
		self.facial_gestures.sad()

	def default(self):
		self.eye_emotions.color_palette1()
		self.facial_gestures.default()
		
	def get_state(self):
		return self.actual_expresion

if __name__ == '__main__':
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	hw_controller = HeadHWController(dxl, dev_id = DEV_ID)
	servos_hw = ServosHW(hw_controller)
	facial_gestures = FacialGestures(servos_hw)
	eyes = EyeEmotion(hw_controller)
	facial_expressions = FacialExpressions(eyes, facial_gestures)
	while True:
		facial_expressions.surprised()
		sleep(2)
		facial_expressions.angry()
		sleep(2)
		facial_expressions.happy()
		sleep(2)
		facial_expressions.sad()
		sleep(2)
		facial_expressions.default()
		sleep(2)
