#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
import os
import sys
from lib_robotis import USB2Dynamixel_Device, Robotis_Servo
from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

class DynamixelCheck(SystemCheckTask):
    """Dynamixel motor hardware check"""
    def __init__(self, device, baud, ids):
        super(DynamixelCheck, self).__init__()
        self.device = device
        self.baud = baud
        self.ids = ids

    def check(self):
        SystemCheck.print_high("Target device  : " + self.device, 1)
        SystemCheck.print_high("Target baudrate: %d" % self.baud, 1)
        try:
            SystemCheck.print_info("Creating USB2Dynamixel interface ... ", 1)
            dxl = USB2Dynamixel_Device(self.device, self.baud)
        except:
            SystemCheck.print_error('Error opening: {} [baud {}]\t\t[FAIL]'
                .format(self.device, self.baud))
            return False

        SystemCheck.print_info("Checking ids  : %s" % self.ids, 1)
        dxl.servo_dev.setTimeout(0.1)
        result = True
        for i in self.ids:
            try:
                Robotis_Servo(dxl, i)
                SystemCheck.print_ok('motor id: {}'.format(i), 2)
            except:
                SystemCheck.print_error('motor id: {} not found'.format(i), 2)
                result = False
        return result
