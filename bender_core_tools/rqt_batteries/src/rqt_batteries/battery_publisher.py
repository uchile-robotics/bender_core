#!/usr/bin/env python

from threading import Thread, Lock
import rospy
from sensor_msgs.msg import BatteryState

class BatteryStatePublisher(object):
    """Basic battery state publisher"""
    def __init__(self, topic='battery_states', serial_number='battery', rate=1):
        # Publisher for battery monitor
        self.batt_pub = rospy.Publisher(topic, BatteryState, queue_size=5)
        # Set serial number
        self.serial_number = serial_number
        # Rate, 1 Hz defaut
        self.rate_freq = rate
        self.rate_pub = rospy.Rate(self.rate_freq)
        # Fill message
        self.msg = BatteryState()
        self.msg.serial_number = self.serial_number
        self.msg.present = True
        # Thread
        self.msg_lock = Lock()
        self.running = True
        self.thread = Thread(target=self.publish_state)
        self.thread.start()
    
    def set_percentage(self, percentage):
        with self.msg_lock:
            # Check overvoltage
            if percentage > 100.0:
                percentage = 100.0
                self.msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
            else:
                self.msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            # Check downvoltage
            if percentage < 0.0:
                percentage = 0.0
                self.msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            else:
                self.msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            # Set percentage
            self.msg.percentage = percentage
            # Update time stamp
            self.msg.header.stamp = rospy.Time.now()

    def set_charging(self, charging):
        with self.msg_lock:
            if charging:
                self.msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                self.msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            # Update time stamp
            self.msg.header.stamp = rospy.Time.now()

    def publish_state(self):
        while self.running and not rospy.is_shutdown():
            with self.msg_lock:
                # Check timeout
                if (rospy.Time.now() - self.msg.header.stamp) < rospy.Duration(5.0):
                    self.batt_pub.publish(self.msg)
            self.rate_pub.sleep()

    def stop(self):
        self.running = False

class VoltageBatteryStatePublisher(BatteryStatePublisher):
    """Battery percentage proportional to voltage"""
    def __init__(self, topic='battery_states', serial_number='battery', rate=1, 
            min_voltage=11.0, max_voltage=13.0):
        super(VoltageBatteryStatePublisher, self).__init__(topic, serial_number, rate)
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage

    def set_voltage(self, voltage):
        self.set_percentage(100*(voltage-self.min_voltage)/(self.max_voltage-self.min_voltage))
