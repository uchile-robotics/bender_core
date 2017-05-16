#!/usr/bin/env python

import rospy
from uchile_rqt_batteries.battery_publisher import VoltageBatteryStatePublisher
from dynamixel_msgs.msg import MotorStateList

class DynamixelBatteryStatePublisher(VoltageBatteryStatePublisher):
    """Battery voltage for Dynamixel motors (percentage proportional to voltage)"""
    def __init__(self, topic='battery_states', serial_number='battery', rate=1, min_voltage=14.8, max_voltage=16.2):
        super(DynamixelBatteryStatePublisher, self).__init__(topic, serial_number, rate, min_voltage, max_voltage)
        # Motor list subscriber
        self.voltage_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_voltage)
    
    def process_voltage(self, msg):
        # Check motor list
        if not msg.motor_states:
            return
        # Average voltage
        sum_voltage = 0.0
        for motor in msg.motor_states:
            sum_voltage += motor.voltage
        voltage = sum_voltage/len(msg.motor_states)
        # Proportional voltage
        self.set_voltage(voltage)

def main():
    rospy.init_node('dxl_battery_publisher', anonymous=True)
    # Get serial number
    serial_number = rospy.get_param('~serial_number', 'dxl_battery')
    # Get battery params
    min_voltage = rospy.get_param('~min_voltage', 14.8)
    max_voltage = rospy.get_param('~max_voltage', 16.2)
    # Battery publisher
    bat_pub = DynamixelBatteryStatePublisher(serial_number=serial_number, min_voltage=min_voltage,
        max_voltage=max_voltage)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
