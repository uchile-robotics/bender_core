#!/usr/bin/env python

import roslib
import rospy
import actionlib
import numpy as np
from scipy import stats
from hark_msgs.msg import HarkSource, HarkSourceVal

from bender_ssl.msg import LocateAction, LocateResult

class LocateSoundServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('locate_sound', LocateAction, self.execute, False)
		self.server.start()
		self.sub = rospy.Subscriber('hark_source', HarkSource, self.hark_source_callback)
		self.hark_data = HarkSource()
		self.x_coordinates = []
		self.y_coordinates = []
		self.angles = []
		self.x_final = 0.0
		self.y_final = 0.0
		self.record = False
		self.no_speech = True
		self._no_speech_count=0
		self.final_result = LocateResult()

	def execute(self, goal):
		# Do lots of awesome groundbreaking robot stuff here
		self.record = True
		self.locate_sound()

	def hark_source_callback(self,msg):
		#save data
		if not self.record:
			return 
		self.hark_data = msg
		if not self.hark_data.src and not self.no_speech:
			self._no_speech_count += 1
			if self._no_speech_count == 10:
				self.no_speech = True
				self._no_speech_count = 0
		if self.hark_data.src and self.no_speech:
			self.no_speech = False


	def locate_sound(self):
		while self.no_speech and not rospy.is_shutdown():
			pass
		while not rospy.is_shutdown():
			if self.no_speech:
				if (len(self.x_coordinates) < 10):
					rospy.loginfo("Nobody is talking to me :(")
					self.x_coordinates = []
					self.y_coordinates = []
					self.angles = []
				else:
					rospy.loginfo("Finding him/her/it")
					rospy.loginfo("I'm not asumming any gender")
					self.x_final = self.calcProm(self.x_coordinates)
					self.y_final = self.calcProm(self.y_coordinates)
					self.angle_final = self.calcProm(self.angles)
					self.angle_final = np.arctan2(self.y_final,self.x_final)
					if self.angle_final >= 0:
						self.angle_final = self.angle_final*180.0/np.pi 
					else:
						self.angle_final = ((2*np.pi)+self.angle_final)*180.0/np.pi
					self.x_coordinates = []
					self.y_coordinates = []
					self.angles = []
					marcianitorealangle = LocateResult()
					marcianitorealangle.final_angle = self.angle_final 
					self.server.set_succeeded(marcianitorealangle)
					self.record = False
					return
			else:
				rospy.loginfo("Somebody is talking...")
				print self.hark_data.src
				if len(self.hark_data.src) == 0:
					continue
				else:
					self.x_coordinates.append(self.hark_data.src[0].x)
					self.y_coordinates.append(self.hark_data.src[0].y)
					self.angles.append(np.arctan2(self.hark_data.src[0].x,self.hark_data.src[0].y))

	def calcProm(self, x):
		dv_permitido = 0.01
		data = x
		moda=stats.mode(data)[0][0]
		dataNew = []
		for i in data:
			if moda+dv_permitido > i and i > moda-dv_permitido:
				dataNew.append(i)
		data = dataNew
		prom=np.mean(data)
		return prom
		
if __name__ == '__main__':
	rospy.init_node('do_dishes_server')
	server = LocateSoundServer()
	rospy.spin()
