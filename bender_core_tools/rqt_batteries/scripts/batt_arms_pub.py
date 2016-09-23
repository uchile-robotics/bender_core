#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import BatteryState
from dynamixel_msgs.msg import MotorStateList,MotorState

charging = None
percentage = None

def get_battery_state(msg):
	global charging
	global percentage
	charging = False
	
	#Buscar numero de motores
	tam=len(msg.motor_states)
	rospy.loginfo(tam)
	rospy.loginfo(msg.motor_states[0])

	#Sacar voltage promedio de motores
	sumvolt=0

	for i in range(tam):
		voltage=msg.motor_states[i].voltage
		sumvolt=sumvolt+voltage
		rospy.loginfo(voltage)

	promvolt=sumvolt/tam		

	percentage = ((promvolt-14.8)/1.4)*100
	rospy.loginfo(percentage)


	#return (charging, percentage)

def main():
	global charging
	global percentage
	rospy.init_node('notebook_battery_publisher', anonymous=True)
	r = rospy.Rate(1) # 1hz

	serial_number = rospy.get_param('~serial_number', 'arm_left')

	rospy.Subscriber("motor_states", MotorStateList, get_battery_state)

	msg = BatteryState()
	msg.serial_number = serial_number
	msg.present = True

	pub = rospy.Publisher('battery_states', BatteryState, queue_size=5)

	while not rospy.is_shutdown():
		#charging, percentage = get_battery_state()
		if charging:
			msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
		else:
			msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
		msg.percentage = percentage
		rospy.logdebug("Current battery state, Charging: {}, Percentage: {}%".format(charging, percentage))
		pub.publish(msg)
		r.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: 
		pass
