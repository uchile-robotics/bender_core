#!/usr/bin/env python

import rospy
from uchile_turning_base.msg import platoMov

class TurningBase(object):
    """Base for bender"""
    def __init__(self, sleep_factor = 1.0):
        self.pub = rospy.Publisher('toggle_servo', platoMov, queue_size=10)
        self.msg = platoMov()
        self.sleep_factor = sleep_factor

    def turn(self, angle, calibration=7000):
        rospy.loginfo("Moving {} degrees".format(angle))
        self.msg.angle = angle
        self.msg.revolution_time = calibration
        self.pub.publish(self.msg)
        rospy.sleep(angle/40.0*self.sleep_factor)
