#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_batteries: batteries_dashboard.py
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Sammy Pfeiffer

import rospy
from rqt_robot_dashboard.dashboard import Dashboard
from python_qt_binding.QtCore import QSize
from sensor_msgs.msg import BatteryState

from .wrap_battery import WrappedBattery

class BatteriesDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Batteries Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._widget_initialized = False

        self.rate = rospy.get_param('~rate',1.0) 
        
        self._batteries_list = dict()
        battery_list = rospy.get_param('~batteries',[]) 
        if not battery_list:
            rospy.logwarn('No batteries to monitor found in param server under \'{}batteries\''.format(rospy.get_namespace()))

        for battery in battery_list:
            rospy.loginfo('Adding \'{}\' to battery monitor'.format(battery))
            self._batteries_list[battery] = WrappedBattery(self.context, name=battery)

        rospy.loginfo('Subscribed to \'{}battery_states\' for battery status'.format(rospy.get_namespace()))
        self._sub = rospy.Subscriber('battery_states', BatteryState, self.dashboard_callback, queue_size=5)

        self._widget_initialized = True


    def get_widgets(self):
        return [self._batteries_list.values()]

    def dashboard_callback(self, msg):
        """
        callback to process messages

        :param msg:
        :type msg: BatteryState
        """
        if not self._widget_initialized:
            return
        # Check battery
        if not msg.serial_number in self._batteries_list:
            return

        # Throttling to 1Hz the update of the widget whatever the rate of the topics is, maybe make it configurable?
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0/self.rate):
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update widget
        charging_status = False
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            charging_status = True
        self._batteries_list[msg.serial_number].set_power_state_perc(msg.percentage, charging_status)


    def shutdown_dashboard(self):
        # Unregister subscriber
        self._sub.unregister()
