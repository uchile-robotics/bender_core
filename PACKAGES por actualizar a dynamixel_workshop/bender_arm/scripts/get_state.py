#!/usr/bin/env python

import sys
import rospy

from sensor_msgs.msg import JointState

subscribe_once = None

class get_state():

	def __init__(self,arm_name):

		self.name = arm_name
		global subscribe_once
		subscribe_once = rospy.Subscriber('/bender/'+self.name+'/joint_states',JointState,self.get_arm_state)
		self.joint_states = JointState()
		self.fo = open('joint_states.txt','a+')

	def get_arm_state(self,data):
		self.joint_states = data 

	def write_position(self):

		current_state = []
		for pos in self.joint_states.position:
			current_state.append(round(pos,3))

		print 'Posicion guardada: ', str(current_state)

		self.fo.write(str(current_state)+'\n')

	def close_file(self):
		self.fo.close

def main():
	state = get_state('l_arm')
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		raw_input('Presiona Enter para guardar Posicion...')
		state.write_position()
		rate.sleep()

	state.close_file()


if __name__ == '__main__':

	rospy.init_node('get_arm_state')
	main()