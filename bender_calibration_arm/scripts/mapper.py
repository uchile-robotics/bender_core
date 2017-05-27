#! /usr/bin/env python
import thread
import rospy 
import rosbag
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
class JointMapper:
	def __init__(self):
		self.pub = rospy.Publisher('calibration_joint_states',JointState,queue_size=10)
		self.sub = rospy.Subscriber('bender/joint_states',JointState,self.callback)
		self.act_sub = rospy.Subscriber('save_joint_state',Empty,self.activate)
		self.flag_ = False
		self.bag = rosbag.Bag(raw_input("Name of bag"), 'w')
	def callback(self,msg):
		if self.flag_:
			self.flag_= False
			self.bag.write('calibration_joint_states',msg)
	def activate(self,msg):
		self.flag_ = True
if __name__ == '__main__':
	rospy.init_node('mapper')
	mapper = JointMapper()
	thread.start_new_thread( rospy.spin,())
	while not rospy.is_shutdown():
		if raw_input('Press enter to save position')=="q":
			mapper.bag.close()
			print "hola"
			break
		mapper.flag_=True

