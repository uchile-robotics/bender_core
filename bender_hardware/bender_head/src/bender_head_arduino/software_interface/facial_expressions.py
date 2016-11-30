#!/usr/bin/python

import roslib; roslib.load_manifest('bender_head')
import time

# NON ROS HARDWARE INTERFACE

"""This is a simple interface to build complete expressions, considering eye's emotion, defined in EyeEmotion class,
and facial gestures defined in FacialGestures class. This interface is used for ros interface."""

# Use HW controllers
from head_hw_controller import HeadHWController
from servos_hw import ServosHW
from eye_emotions import EyeEmotion
from facial_gestures import FacialGestures

class FacialExpressions(object):
	def __init__(self, eye_emotions, facial_gestures):
		self.eye_emotions = eye_emotions
		self.facial_gestures = facial_gestures
		self.actual_expresion = ""

	def surprised(self):
		self.eye_emotions.surprised()
		self.facial_gestures.surprised()
		self.actual_expresion = "surprised"

	def angry(self):
		self.eye_emotions.angry()
		self.facial_gestures.angry()
		self.actual_expresion = "angry"

	def happy(self):
		self.eye_emotions.happy()
		self.facial_gestures.happy()
		self.actual_expresion = "happy"

	def sad(self):
		self.eye_emotions.sad()
		self.facial_gestures.sad()
		self.actual_expresion = "sad"
		
	def veryHappy(self):
		self.eye_emotions.happy()
		self.facial_gestures.veryHappy()
		self.actual_expresion = "veryHappy"

	def default(self):
		self.eye_emotions.color_palette1()
		self.facial_gestures.default()
		self.actual_expresion = "default"

	def apagado(self):
		self.eye_emotions.apagado()
		self.facial_gestures.default()
		self.actual_expresion = "apagado"
		
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
		time.sleep(2)
		facial_expressions.angry()
		time.sleep(2)
		facial_expressions.happy()
		time.sleep(2)
		facial_expressions.sad()
		time.sleep(2)
		facial_expressions.default()
		time.sleep(2)
