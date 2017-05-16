#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import BatteryState
from uchile_rqt_batteries.battery_publisher import BatteryStatePublisher

class NotebookBatteryPublisher(BatteryStatePublisher):
	"""Publish notebook battery state"""
	def __init__(self, topic='battery_states', serial_number='battery', rate=1):
		super(NotebookBatteryPublisher, self).__init__(topic, serial_number, rate)	

	def get_battery_state(self):
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

	def spin(self):
		while not rospy.is_shutdown():
			charging, percentage = self.get_battery_state()
			self.set_charging(charging)
			self.set_percentage(percentage)
			rospy.sleep(1.0)

def main():
	rospy.init_node('notebook_battery_publisher', anonymous=True)
	# Get serial number
	serial_number = rospy.get_param('~serial_number', 'notebook')
	# Battery publisher
	bat_pub = NotebookBatteryPublisher(serial_number=serial_number)
	bat_pub.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: 
		pass
