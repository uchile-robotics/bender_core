#!/usr/bin/env python
import subprocess
import rospy
from std_msgs.msg import Float32, Bool

def get_battery_state():
	cmd = "upower -i $(upower -e | grep 'BAT') | grep -E 'state|to\ full|percentage'"
	ps = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
	output = ps.communicate()[0]

	conectado=True
	if 'discharging' in output:
		conectado=False

	posicion=output.index('%')
	porcentaje=output[posicion-3:posicion]

	if porcentaje[0]==' ':
		porcentaje=porcentaje[1:3]

	porcentaje = float(porcentaje)

	return (conectado, porcentaje)


def main():
	rospy.init_node('notebook_battery_publisher', anonymous=True)
	r = rospy.Rate(1) # 1hz

	bat_number =  '1'
	percentage_topic = '/percentage' + bat_number
	charging_topic = '/charging' + bat_number

	# Config dashboard
	batteries_list = list()
	batteries_list.append({'battery1' : {'percentage_topic': percentage_topic, 'charging_topic': charging_topic, 'tooltip_name': 'notebook'}})
	rospy.loginfo('Setting params.')
	rospy.set_param('/batteries_dashboard/batteries', batteries_list)
	
	pub_percentage = rospy.Publisher(percentage_topic, Float32, queue_size=10)
	pub_charging = rospy.Publisher(charging_topic, Bool, queue_size=10)

	percentage_msg = Float32()
	charging_msg = Bool()
	while not rospy.is_shutdown():
		charging_msg.data, percentage_msg.data = get_battery_state()
		rospy.loginfo("Current battery state, Charging: {}, Percentage: {}%".format(charging_msg.data, percentage_msg.data))
		pub_percentage.publish(percentage_msg)
		pub_charging.publish(charging_msg)
		r.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: 
		pass
