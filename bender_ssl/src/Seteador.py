#!/usr/bin/env python
import rospy
import numpy as np
from scipy import stats
from std_msgs.msg import Float32
from hark_msgs.msg import HarkSource, HarkSourceVal, promxy

class MicroconeMark(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/hark_source', HarkSource, self.set_callback)
		self.pub = rospy.Publisher('/Seteador', Float32, queue_size=10)
		self.powermediciones = []
		self.primero = True
		self.set = 26.0


	def set_callback(self, msg):
		if (self.primero):
			if (len(self.powermediciones) < 1500):
				self.powermediciones.append(msg.src[0].power)
				self.pub.publish(self.set)
			else:
				data = self.powermediciones
				self.set = np.mean(stats.mode(data)[0])
				self.pub.publish(self.set)
				self.primero = False
				self.powermediciones = []
		else:
			if (len(self.powermediciones) < 50):
				self.powermediciones.append(msg.src[0].power)
				self.pub.publish(self.set)
				self.primero = False
			else:
				data = self.powermediciones
				self.set = np.mean(stats.mode(data)[0])
				self.pub.publish(self.set)
				self.primero = False
				self.powermediciones = []

def main():
	rospy.init_node('base_controller2')
	rospy.loginfo('Init base controller2')
	base = MicroconeMark()   
	rospy.spin()

if __name__ == '__main__':
	main()
