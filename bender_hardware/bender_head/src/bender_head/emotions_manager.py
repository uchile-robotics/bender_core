#!/usr/bin/python

import rospy
import time

# NON ROS HARDWARE INTERFACE

"""This is a simple interface to build complete expressions, considering eye's emotion, defined in EyeEmotion class,
and facial gestures defined in FacialGestures class. This interface is used for ros interface."""

# Use HW controllers
from head_hw import HeadHW
from servos_hw import ServosHW

class EmotionsManager(object):
	def __init__(self, hw_controller, servos_hw):
		self.hw_controller = hw_controller
		self.servos_hw = servos_hw
		self.actual_expresion = ""
		# Check param
		rospy.logwarn(rospy.get_namespace())
		self.emotions = rospy.get_param('emotions')
		self.dynamic_emotions = rospy.get_param('dynamic_emotions')
		self.colors = rospy.get_param('eye_colors')

	def get_rgb_colors(self, str_colors):
		rgb_colors = []
		for color in str_colors:
			rgb_color = self.colors[color]
			rgb_colors.append(rgb_color)
		return rgb_colors

	def moveNeck(self, angle):		
		self.servos_hw.neck(angle)

	def set_emotion(self, emotion):
		emo = self.emotions[emotion]
		left_eye_colors = self.get_rgb_colors(emo['eyes']['left_eye'])
		right_eye_colors = self.get_rgb_colors(emo['eyes']['right_eye'])
		self.hw_controller.set_eye_colors("left", left_eye_colors)
		self.hw_controller.set_eye_colors('right', left_eye_colors)

		if (emo['servos']!='NonUsed'):
			self.servos_hw.left_ear(emo['servos']['left_ear'])
			self.servos_hw.right_ear(emo['servos']['right_ear'])
			self.servos_hw.left_eyebrow(emo['servos']['left_eyebrow'])
			self.servos_hw.right_eyebrow(emo['servos']['right_eyebrow'])
			self.servos_hw.mouth(emo['servos']['mouth'])
			
		self.actual_expresion = emotion

	def set_dynamic_emotion(self, emotion):
		emo = self.dynamic_emotions[emotion]
		if emo['left_eye'] != 'NonUsed':
			left_eye_colors = self.get_rgb_colors(emo['left_eye'])
			self.hw_controller.set_eye_colors('left', left_eye_colors)
		if emo['right_eye'] != 'NonUsed':
			right_eye_colors = self.get_rgb_colors(emo['right_eye'])
			self.hw_controller.set_eye_colors('right', left_eye_colors)
		
		movements = emo['sizes']
		max_iter = max(movements)
		for i in range(max_iter):
			if(i<movements[0] and emo['left_ear']!='NonUsed'):
				self.servos_hw.left_ear(emo['left_ear'][i])
			if(i<movements[1] and emo['right_ear']!='NonUsed'):
				self.servos_hw.right_ear(emo['right_ear'][i])
			if(i<movements[2] and emo['left_eyebrow']!='NonUsed'):
				self.servos_hw.left_eyebrow(emo['left_eyebrow'][i])
			if(i<movements[3] and emo['right_eyebrow']!='NonUsed'):
				self.servos_hw.right_eyebrow(emo['right_eyebrow'][i])
			if(i<movements[4] and emo['mouth']!='NonUsed'):
				self.servos_hw.mouth(emo['mouth'][i])
			time.sleep(int(emo['time'])/1000.0)
	

	def get_state(self):
		return self.actual_expresion

# if __name__ == '__main__':
# 	DEV_ID = 1
# 	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
# 	hw_controller = HeadHW(dxl, dev_id = DEV_ID)
# 	servos_hw = ServosHW(hw_controller)
# 	emotions_controller = EmotionsManager(hw_controller, servos_hw)
# 	test_emotions = ['sad', 'surprised', 'angry', 'happy', 'apagado']
# 	for emotion in test_emotions:
# 		emotions_controller.set_emotion(emotion)
# 		time.sleep(3)