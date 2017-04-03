#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
import os
import sys
from lib_robotis import USB2Dynamixel_Device, Robotis_Servo
from bender_utils.syscheck import SystemCheck, SystemCheckTask, FileCheckTask

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
        try:
            dxl = USB2Dynamixel_Device(self.device, self.baud)
        except:
            SystemCheck.print_error('Error opening: {} [baud {}]\t\t[FAIL]'
                .format(self.device, self.baud))
            return False
        dxl.servo_dev.setTimeout(0.1)
        result = True
        for i in self.ids:
            try:
                Robotis_Servo(dxl, i)
                SystemCheck.print_ok('\tMotor ID: {}\t\t[OK]'.format(i))
            except:
                SystemCheck.print_error('\tMotor ID: {} not found\t\t[FAIL]'
                    .format(i))
                result = False
        return result


def main():
    # Check l_arm
    l_arm = SystemCheck()
    l_arm.add_child(FileCheckTask('/dev/bender/l_port'))
    l_arm.add_child(DynamixelCheck('/dev/bender/l_port', 200000,
        [1, 2, 3, 4, 5, 6, 7, 8]))
    l_arm_res = l_arm.check()
    # Check r_arm
    r_arm = SystemCheck()
    r_arm.add_child(FileCheckTask('/dev/bender/r_port'))
    r_arm.add_child(DynamixelCheck('/dev/bender/r_port', 200000,
        [11, 12, 13, 14, 15, 16, 17, 18]))
    r_arm_res = r_arm.check()
    
    # Return
    SystemCheck.exit(l_arm_res and r_arm_res)

if __name__ == "__main__":
    main()
