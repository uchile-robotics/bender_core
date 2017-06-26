#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
from lib_robotis import USB2Dynamixel_Device, Robotis_Servo
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
            # Send zeros for restart all state machines on servos
            dxl.__write_serial([0]*20)
            sleep(0.1)
            dxl.__write_serial([0]*20)

        except:
            SystemCheck.print_error("Error opening: {} [baud {}]\t\t[FAIL]"
                .format(self.device, self.baud))
            return False

        SystemCheck.print_info("Checking ids  : %s" % self.ids, 1)
        result = True
        for i in self.ids:
            try:
                result = dxl.ping(i)
                if result:
                    SystemCheck.print_ok("Motor id: {}".format(i), 2)
                else:
                    raise RuntimeError("Motor id:{} not found".format(i))
            except:
                SystemCheck.print_error("Motor id: {} not found".format(i), 2)
                result = False
        try:
            dxl.close()
        except:
            SystemCheck.print_error("Error closing port.")
            result = False
        return result
