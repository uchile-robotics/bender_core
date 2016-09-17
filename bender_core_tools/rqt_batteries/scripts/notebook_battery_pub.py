#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import BatteryState

def get_battery_state():
	cmd = "upower -i $(upower -e | grep 'BAT') | grep -E 'state|to\ full|percentage'"
	ps = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
	output = ps.communicate()[0]
	# Get status
	charging=True
	if 'discharging' in output:
		charging=False
	# Get percentage
	percentage = 0.0
	try:
		percentage_pos=output.index('%')
		percentage=float(output[percentage_pos-3:percentage_pos].strip())
	except ValueError:
		pass
	return (charging, percentage)

def main():
	rospy.init_node('notebook_battery_publisher', anonymous=True)
	r = rospy.Rate(1) # 1hz

	serial_number = rospy.get_param('~serial_number', 'base')

	msg = BatteryState()
	msg.serial_number = serial_number
	msg.present = True

	pub = rospy.Publisher('battery_states', BatteryState, queue_size=5)

	while not rospy.is_shutdown():
		charging, percentage = get_battery_state()
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
