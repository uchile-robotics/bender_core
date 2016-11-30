#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from dynamixel_msgs.msg import MotorStateList,MotorState

# float64 timestamp   # motor state is at this time
#int32 id            # motor id
#int32 goal          # commanded position (in encoder units)
#int32 position      # current position (in encoder units)
#int32 error         # difference between current and goal positions
#int32 speed         # current speed (0.111 rpm per unit)
#float64 load        # current load - ratio of applied torque over maximum torque
#float64 voltage     # current voltage (V)
#int32 temperature   # current temperature (degrees Celsius)
#bool moving         # whether the motor is currently in motion


def fill_msg():
	motor_list = MotorStateList()
	for i in range(5):
		motor_state = MotorState()
		motor_state.voltage = 16.2 - i/5.0
		motor_list.motor_states.append(motor_state)
	return motor_list

def main():
	rospy.init_node('motor_state_test', anonymous=True)
	r = rospy.Rate(1)
	pub = rospy.Publisher('motor_states', MotorStateList, queue_size=10)

	while not rospy.is_shutdown():
		data = fill_msg()
		pub.publish(data)
		r.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass