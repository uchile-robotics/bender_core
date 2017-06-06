#!/usr/bin/env python
import rospy
import numpy as np
from scipy import stats
from std_msgs.msg import Float32
from hark_msgs.msg import HarkSource, HarkSourceVal, promxy

class MicroconeMark(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/hark_source', HarkSource, self.prom_callback)
		self.pub = rospy.Publisher('/talking_topic', promxy, queue_size=10)
		self.xy = promxy()
		self.xmediciones = []
		self.ymediciones = []


	def prom_callback(self, msg):
		if not msg.src:
			if (len(self.xmediciones) < 10):
				rospy.loginfo("Nobody is talking to me :(")
				self.xmediciones = []
				self.ymediciones = []
			else:
				rospy.loginfo("Finding him/her/it")
				rospy.loginfo("I'm not asumming any gender")
				self.xy.x = self.calcProm(self.xmediciones)
				self.xy.y = self.calcProm(self.ymediciones)
				self.xmediciones = []
				self.ymediciones = []
				self.pub.publish(self.xy)
		else:
			rospy.loginfo("Somebody is talking...")
			self.xmediciones.append(msg.src[0].x)
			self.ymediciones.append(msg.src[0].y)
			self.xy.x = 1.0
			self.xy.y = 0.0

	def calcProm(self, x):
		dv_permitido = 5.0
		data = x
		moda=stats.mode(data)[0][0]
		dataNew = []
		for i in data:
			if moda+dv_permitido > i and i > moda-dv_permitido:
				dataNew.append(i)
		data = dataNew
		prom=np.mean(data)
		return prom	

def main():
	rospy.init_node('base_controller2')
	rospy.loginfo('Init base controller2')
	base = MicroconeMark()   
	rospy.spin()

if __name__ == '__main__':
	main()
