#!/usr/bin/python

import time
#from dynamixel_driver.dynamixel_io import DynamixelIO
from dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

SERVO_SELECT_STATE = 6 # memory address of variable servo_selct in driver hardware
SERVO_POS_STATE = 7 # memory address of variable servo_pos in driver hardware
SERVO0 = 0
SERVO1 = 1
SERVO2 = 2
SERVO3 = 3
SERVO4 = 4
LED_SELECT_STATE = 8
LED_COLOR_STATE = 9

#colores
global black, red, green, blue
black = 1
red = 2
green = 3
blue = 4

class HeadHW(object):
	#red = 
	def __init__(self, dxl_io, dev_id = 1):
		self.dxl = dxl_io
		self.id = dev_id
		self.state = [0]

	def ping(self):
		result = []
		try:
			result = self.dxl.ping(self.id)
		except Exception as e:
			print 'Exception thrown while pinging device %d - %s' % (self.id, e)
			raise e
		return result

	def get_state(self, state_variable):
		result = []
		try:
			result = self.dxl.read(self.id, state_variable, 1)
		except Exception as e:
			print 'Exception thrown while reading addres %d' % (state_variable)
			return e
		if (state_variable == LED_COLOR_STATE): self.state = [(result[5] & int('0b00110000',2))>>4, (result[5] & int('0b00001100',2))>>2, (result[5] & int('0b00000011',2))]
		else: self.state = [result[5]]
		return self.state

	def select_command(self, com = 1):
		if (com != SERVO0) and (com != SERVO1) and (com != SERVO2) and (com != SERVO3) and (com != SERVO4) and (com != 5):
			print 'Unknown command %d' % (com)
		
		result = []
		try:
			result = self.dxl.write(self.id, SERVO_SELECT_STATE, [com])
		except Exception as e:
			print 'Exception thrown while writing addres %d' % (SERVO_SELECT_STATE)
			raise e
		return result

	def pos_command(self, com = 0):
		if (com < 0) or (com > 180):
			print 'command %d out of range' % (com)
			#return
		result = []
		try:
			result = self.dxl.write(self.id, SERVO_POS_STATE, [com])
		except Exception as e:
			print 'Exception thrown while writing addres %d' % (SERVO_SELECT_STATE)
			raise e
		return result

	def moveServoTo(self, servo_i, pos):
		self.select_command(servo_i)	#update servo select value
		self.pos_command(pos)			#update servo position value
		self.select_command(0x05)		#when both states updated, confirm the change by sending the code '0x05'
		return

	def swapServo(self, servo_i):
		for i in range(180): self.moveServoTo(servo_i, i)
		for i in range(180,0,-1): self.moveServoTo(servo_i, i)

	def parallelSwapServos(self):
		for pos in range(0,180,10):
			for servo_i in range(5): self.moveServoTo(servo_i, pos)

		for pos in range(180,0,-10):
			for servo_i in range(5): self.moveServoTo(servo_i, pos)

	def updateLedColor(self, numLed, r_color, g_color, b_color):
		if (numLed < 0) or (numLed > 36):
			print 'command %d out of range' % (com)
			#return
		result = []
		color_code = (r_color<<4) | (g_color<<2) | b_color
		try:
			result1 = self.dxl.write(self.id, LED_SELECT_STATE, [numLed])	# select LED i-esimo
			time.sleep(0.001)
			result2 = self.dxl.write(self.id, LED_COLOR_STATE, [color_code]) 	# send new color_code of LED numLed
			time.sleep(0.001)
			result3 = self.dxl.write(self.id, LED_SELECT_STATE, [0xFD]) 	# confirm change in color
			time.sleep(0.001)
		except Exception as e:
			print 'Exception thrown while writing addres %d' % (LED_SELECT_STATE)
			raise e
		#return result1 & result2 & result3
		return result3

	def updateLedColor_ready(self):
		result = []
		try:
			result1 = self.dxl.write(self.id, LED_SELECT_STATE, [0xFE])	# this command shows the color set by updateLedColor
			time.sleep(0.01)
			#result2 = self.dxl.write(self.id, LED_SELECT_STATE, [0xFC])	# this command ends the change in LEDs rings
		except Exception as e:
			print 'Exception thrown while writing addres %d, command 0xFE' % (LED_SELECT_STATE)
			raise e
		try:
			#result1 = self.dxl.write(self.id, LED_SELECT_STATE, [0xFE])	# this command shows the color set by updateLedColor
			result2 = self.dxl.write(self.id, LED_SELECT_STATE, [0xFC])	# this command ends the change in LEDs rings
			time.sleep(0.01)
		except Exception as e:
			print 'Exception thrown while writing addres %d, command 0xFC' % (LED_SELECT_STATE)
			raise e
		#return result1 & result2
		return result2

	def changeLedColor(self, numLed, rgb_color):
		self.updateLedColor(numLed, rgb_color[0], rgb_color[1], rgb_color[2])
		self.updateLedColor_ready()

	def set_eye_colors(self, eye, rgb_colors):
		if (eye!="left" and eye!="right"):
			print "parameter eye must be <left> or <right>, <%s> given" %(eye)
			return
		if len(rgb_colors)!=16:
			print "bad number of colors: %d. Must be 16" %(len(colors))
			return
		if (eye=="left"):
			#print "left"
			first_led = 0
			last_led = 16
		else:
			#print "right"
			first_led = 16
			last_led = 32
		for i_led in range(first_led,last_led,1):
			r_color = rgb_colors[i_led-16][0]
			g_color = rgb_colors[i_led-16][1]
			b_color = rgb_colors[i_led-16][2]
			self.updateLedColor(i_led, r_color, g_color, b_color)
		self.updateLedColor_ready()

	def set_this_leds_to(self, leds, rgb_colors):
		if (len(leds)!=len(rgb_colors)):
			print "The number of LEDs must be the same as the number of colors. %d leds and %d colors given"
			return
		for i_led in range(len(leds)): self.updateLedColor(leds[i_led], rgb_colors[i_led][0], rgb_colors[i_led][1], rgb_colors[i_led][2])
		self.updateLedColor_ready()

	def sendSurprised(self): #implement with 'set_eye_colors'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		rgb_colors = [blue,blue,blue,red,red,red,black,black,black,blue,red,blue,green,green,green,green]
		self.set_eye_colors("left", rgb_colors)
		self.set_eye_colors("right", rgb_colors)

	def sendAngry(self): #implement with 'set_eye_colors'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		rgb_colors = [green,green,green,blue,blue,blue,blue,blue,blue,blue,blue,blue,green,green,green,green]
		self.set_eye_colors("left", rgb_colors)
		self.set_eye_colors("right", rgb_colors)

	def sendResetColors(self): #implement with 'set_eye_colors'
		black = [0,0,0]
		rgb_colors = [black,black,black,black,black,black,black,black,black,black,black,black,black,black,black,black]
		self.set_eye_colors("left", rgb_colors)
		self.set_eye_colors("right", rgb_colors)

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
		self.set_eye_colors("left", rgb_colors_l)
		self.set_eye_colors("right", rgb_colors_r)

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
		self.set_eye_colors("left", rgb_colors_l)
		self.set_eye_colors("right", rgb_colors_r)

	def sendHappy(self): #implement with 'set_this_leds_to'
		black = [0,0,0]
		red = [3,0,0]
		green = [0,3,0]
		blue = [0,0,3]
		leds_left_eye = [0,1,2,13,14,15]
		leds_right_eye = [16,17,18,29,30,31]
		rgb_colors = [green,green,blue,blue,green,green]
		self.set_this_leds_to(leds_left_eye, rgb_colors)
		self.set_this_leds_to(leds_right_eye, rgb_colors)

if __name__ == '__main__':
	import time
	DEV_ID = 1
	dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
	head = HeadHW(dxl, dev_id = DEV_ID)
	while True:
		#for pos in range(180):
		#head.moveServoTo(SERVO2, pos)
		head.parallelSwapServos()
		head.sendSurprised()
		head.sendAngry()
		#head.sendHappy()
		#head.changeLedColor(0, red)
