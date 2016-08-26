#!/usr/bin/python

import roslib; roslib.load_manifest('bender_head')
import time
import rgb_colors_names as c

# NON ROS HARDWARE INTERFACE

"""In this class possible colors of the LEDs Bender's eyes representing their emotions are defined.
This is done by "set_eye_colors (eye, rgb_colors)" method of the "HeadHWController" class. For more details see Documentation.
Seven standard "emotion methods" for eyes's colors have been given.
Modify to create new emotions based on LEDs colors."""

# Use HW controller
from head_hw_controller import HeadHWController

class EyeEmotion(object):
	def __init__(self, hw_controller):
		self.hw_controller = hw_controller

	def surprised(self):
		rgb_colors = [c.blue,c.blue,c.blue,c.red,c.red,c.red,c.black,c.black,c.black,c.blue,c.red,c.blue,c.green,c.green,c.green,c.green]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def angry(self):
		rgb_colors = [c.red,c.red,c.red,c.red,c.yellow2,c.yellow2,c.black,c.black,c.black,c.red,c.black,c.black,c.green,c.green,c.green,c.green]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)
		
	def sad(self):
		rgb_colors = [c.black,c.black,c.black,c.blue,c.black,c.blue,c.black,c.blue,c.black,c.blue,c.black,c.blue,c.black,c.black,c.black,c.black]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def happy(self):
		rgb_colors = [c.black,c.black,c.black,c.black,c.yellow3,c.yellow3,c.black,c.black,c.black,c.black,c.yellow3,c.yellow3,c.yellow3,c.yellow3,c.black,c.black]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def apagado(self):
		rgb_colors = [c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black,c.black]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def color_palette1(self):
		led = 0
		rgb_colors_l = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		rgb_colors_r = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		for g in range(4):
			for b in range(4):
				rgb_colors_l[led] = [0,g,b]
				led+= 1
		led=0
		for g in range(4):
			for b in range(4):
				rgb_colors_r[led] = [1,g,b]
				led+= 1
		self.hw_controller.set_eye_colors("left", rgb_colors_l)
		self.hw_controller.set_eye_colors("right", rgb_colors_r)

	def color_palette2(self):
		led = 0
		rgb_colors_l = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		rgb_colors_r = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		for g in range(4):
			for b in range(4):
				rgb_colors_l[led] = [2,g,b]
				led+= 1
		led=0
		for g in range(4):
			for b in range(4):
				rgb_colors_r[led] = [3,g,b]
				led+= 1
		self.hw_controller.set_eye_colors("left", rgb_colors_l)
		self.hw_controller.set_eye_colors("right", rgb_colors_r)

if __name__ == '__main__':
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	hw_controller = HeadHWController(dxl, dev_id = DEV_ID)
	eyes = EyeEmotion(hw_controller)
	while True:
		eyes.surprised()
		time.sleep(2)
		eyes.angry()
		time.sleep(2)
		eyes.happy()
		time.sleep(2)
		eyes.color_palette1()
		time.sleep(2)
		eyes.color_palette2()
		time.sleep(2)
		eyes.reset()
		time.sleep(2)
