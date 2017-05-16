#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# uchile_rqt_batteries: batteries_dashboard.py
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
from python_qt_binding.QtWidgets import QGroupBox, QHBoxLayout, QLabel
from python_qt_binding.QtCore import Qt, QSize, Signal
from sensor_msgs.msg import BatteryState

from .wrap_battery import WrappedBattery
from threading import Thread

class BatteriesDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    _sig_add_battery = Signal(str)
    _sig_remove_battery = Signal(str)

    def setup(self, context):
        self.name = 'Batteries Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._widget_initialized = False

        self.rate = rospy.get_param('~rate',1.0) 
        
        self._batteries = dict()

        rospy.loginfo('Subscribed to \'{}battery_states\' for battery status'.format(rospy.get_namespace()))
        self._sub = rospy.Subscriber('battery_states', BatteryState, self.dashboard_callback, queue_size=5)

        self._sig_add_battery.connect(self._add_batteries)
        self._sig_remove_battery.connect(self._remove_batteries)

        self._running = True
        Thread(target=self.check_outdated).start()

        self._widget_initialized = True

    def _add_batteries(self, battery_name):
        """
        Slot for signals that update batteries widgets
        """
        rospy.loginfo('New battery \'{}\''.format(battery_name))
        self._batteries[battery_name] = WrappedBattery(self.context, name=battery_name)
        self.add_widgets()

    def _remove_batteries(self, battery_name):
        """
        Slot for signals that remove batteries widgets
        """
        rospy.loginfo('Deleting battery \'{}\''.format(battery_name))
        del self._batteries[battery_name]
        self.add_widgets()

    def add_widgets(self):
        """
        Add groups of widgets to _main_widget. Supports group labels.
        """
        self._main_widget.clear()
        widgets = self.get_widgets()
        self._widgets = [] # stores widgets which may need to be shut down when done
        for group in widgets:
            # Check for group label
            if isinstance(group[0], str):
                grouplabel, v = group
                box = QGroupBox(grouplabel)
                box.setContentsMargins(0, 18, 0, 0) # LTRB
                # Apply the center-label directive only for single-icon groups
                if len(group[1]) == 1:
                    box.setAlignment(Qt.AlignHCenter)
            else:
                box = QGroupBox()
                box.setContentsMargins(0, 0, 0, 0) # LTRB
                v = group
            # Add widgets to QGroupBox
            layout = QHBoxLayout()
            layout.setSpacing(0)
            layout.setContentsMargins(0, 0, 0, 0) # LTRB
            for i in v:
                try:
                    try:
                        i.setIconSize(self.max_icon_size) # without this, icons are tiny
                    except AttributeError as e:
                        # triggers with battery which uses a QLabel instead of a QToolButton-based widget
                        pass
                    layout.addWidget(i)
                    self._widgets.append(i)
                except:
                    raise Exception("All widgets must be a subclass of QWidget!")

            layout.activate()
            box.setLayout(layout)
            self._main_widget.addWidget(box)
            self._main_widget.addSeparator()

    def get_widgets(self):
        widget_groups = []
        for battery_name in self._batteries.keys():
            widget_groups.append([str(self._batteries[battery_name]._name),[self._batteries[battery_name]]])
        return widget_groups

    def dashboard_callback(self, msg):
        """
        callback to process messages

        :param msg:
        :type msg: BatteryState
        """
        if not self._widget_initialized:
            return
        # Battery name
        battery_name = msg.serial_number
        # Add battery
        if not battery_name in self._batteries:
            # Call Qt UI thread for add new widget
            self._sig_add_battery.emit(battery_name)
            return

        # Update widget
        charging_status = False
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            charging_status = True
        self._batteries[battery_name].set_power_state_perc(msg.percentage, charging_status)


    def check_outdated(self):
        while self._running:  
            try:
                # Check outdated batteries
                for battery in self._batteries.values():
                    if (rospy.Time.now() - battery.last_update) > rospy.Duration(5.0):
                        rospy.logdebug('Battery \'{}\' information is outdated'.format(battery._name))
                        battery.set_power_state_perc(0.0, False, refresh=False)

                    if (rospy.Time.now() - battery.last_update) > rospy.Duration(10.0):
                        rospy.logwarn('Battery \'{}\' will be deleted from dashboard'.format(battery._name))
                        self._sig_remove_battery.emit(battery._name)
                rospy.sleep(1.0)

            except rospy.ROSInterruptException: 
                self._running = False

    def shutdown_dashboard(self):
        # Close thread
        self._running = False
        # Unregister subscriber
        self._sub.unregister()

