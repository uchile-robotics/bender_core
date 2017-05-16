#!/usr/bin/env python
import rospy
from uchile_rqt_batteries.battery_publisher import VoltageBatteryStatePublisher
from std_msgs.msg import Float64

class VoltageFloatBatteryStatePublisher(VoltageBatteryStatePublisher):
    """Battery voltage published as std_msgs/Float64 (percentage proportional to voltage)"""
    def __init__(self, topic='battery_states', serial_number='battery', rate=1, min_voltage=11.0, max_voltage=13.0):
        super(VoltageFloatBatteryStatePublisher, self).__init__(topic, serial_number, rate, min_voltage, max_voltage)
        # Voltage subscriber
        self.voltage_sub = rospy.Subscriber('battery_voltage', Float64, self.process_voltage)
    
    def process_voltage(self, msg):
        # Proportional voltage
        self.set_voltage(msg.data)

def main():
    rospy.init_node('voltage_battery_publisher', anonymous=True)
    # Get serial number
    serial_number = rospy.get_param('~serial_number', 'battery')
    # Get battery params
    min_voltage = rospy.get_param('~min_voltage', 11.0)
    max_voltage = rospy.get_param('~max_voltage', 13.0)
    # Battery publisher
    bat_pub = VoltageFloatBatteryStatePublisher(serial_number=serial_number, min_voltage=min_voltage,
        max_voltage=max_voltage)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
