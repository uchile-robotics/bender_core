#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from dynamixel_driver.dynamixel_io import DynamixelIO
from time import sleep

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
            SystemCheck.print_info("Opening serial port... ", 1)
            dxl = DynamixelIO(self.device, self.baud)
        except:
            SystemCheck.print_error("Error opening: {} [baud {}]\t\t[FAIL]"
                .format(self.device, self.baud))
            return False

        SystemCheck.print_info("Checking ids  : %s" % self.ids, 1)
        result = True
        found_device = set()
        required =set(self.ids)
        for device_id in self.ids:
            for trial in range(5):
                try:
                    result = dxl.ping(device_id)
                except Exception as ex:
                    SystemCheck.print_error("Exception thrown while pinging device id {}.\t\t[FAIL]"
                        .format(device_id))
                    SystemCheck.print_error(str(ex))
                if result:
                    SystemCheck.print_ok("Motor id: {}".format(device_id), 2)
                    found_device.add(device_id)
                    break

        if not required.issubset(found_device):
            not_found = required.difference(found_device)
            for device_id in not_found:
                SystemCheck.print_error("Motor id: {} not found".format(device_id), 2)
            result = False           
        return result
