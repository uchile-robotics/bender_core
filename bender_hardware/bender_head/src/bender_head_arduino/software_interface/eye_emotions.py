#!/usr/bin/python

import time

# NON ROS HARDWARE INTERFACE

# Use HW controller
from head_hw_controller import HeadHWController

class EyeEmotion(object, hw_controller):
	def __init__(self):
		self.hw_controller = hw_controller

	def surprised(self): #implemented with 'set_eye_colors'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		rgb_colors = [blue,blue,blue,red,red,red,black,black,black,blue,red,blue,green,green,green,green]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def angry(self): #implemented with 'set_eye_colors'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		rgb_colors = [green,green,green,blue,blue,blue,blue,blue,blue,blue,blue,blue,green,green,green,green]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)
		
	def sad(self): #implemented with 'set_eye_colors'
		black = [0,0,0]
		blue = [0,0,3]
		rgb_colors = [black,black,black,blue,black,blue,black,blue,black,blue,black,blue,black,black,black,black]
		self.hw_controller.set_eye_colors("left", rgb_colors)
		self.hw_controller.set_eye_colors("right", rgb_colors)

	def happy(self): #implemented with 'set_this_leds_to'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		leds_left_eye = [0,1,2,13,14,15]
		leds_right_eye = [16,17,18,29,30,31]
		rgb_colors = [green,green,blue,blue,green,green]
		self.hw_controller.set_this_leds_to(leds_left_eye, rgb_colors)
		self.hw_controller.set_this_leds_to(leds_right_eye, rgb_colors)

	def reset(self): #implemented with 'set_eye_colors'
		black = [0,0,0]
		rgb_colors = [black,black,black,black,black,black,black,black,black,black,black,black,black,black,black,black]
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
		sleep(2)
		eyes.angry()
		sleep(2)
		eyes.happy()
		sleep(2)
		eyes.color_palette1()
		sleep(2)
		eyes.color_palette2()
		sleep(2)
		eyes.reset()
		sleep(2)
