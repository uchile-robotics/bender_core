#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
import os
import sys
from lib_robotis import USB2Dynamixel_Device, Robotis_Servo

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

class Colors(object):
    """
    Color code for console print
    """
    def __init__(self):
        pass

    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class HardwareCheck(object):
    """
    Base class for hardware check
    """
    def __init__(self):
        self.child = list()

    def add_child(self, child):
        """
        Add children for check
        """
        if not issubclass(type(child), HardwareCheck):
            HardwareCheck.print_error("{} is not derived class from HardwareCheck".format(child.__class__.__name__))
            return
        self.child.append(child)

    def check_children(self):
        complete_result = True
        for child in self.child:
            result = child.check()
            complete_result = complete_result and result
        return result

    def check(self):
        return self.check_children()

    @staticmethod
    def print_error(data):
        """
        Print message in red
        """
        print(Colors.FAIL + data + Colors.END)

    @staticmethod
    def print_ok(data):
        """
        Print message in green
        """
        print(Colors.OKGREEN + data + Colors.END)

    @staticmethod
    def exit_error():
        """
        Exit the script using sys.exit(-1)
        """
        sys.exit(-1)

    @staticmethod
    def exit_ok():
        """
        Exit the script using sys.exit(0)
        """
        sys.exit(0)

    @staticmethod
    def exit(ok=False):
        if not ok:
            HardwareCheck.exit_error()
        else:
            HardwareCheck.exit_ok()


class FileCheck(HardwareCheck):
    """File existence check"""
    def __init__(self, filename):
        super(FileCheck, self).__init__()
        self.filename = filename

    def check(self):
        result = bool(os.path.exists(self.filename))
        if result:
            HardwareCheck.print_ok('{} found'.format(self.filename))
        else:
            HardwareCheck.print_error('{} not found'.format(self.filename))
        return result

class DynamixelCheck(HardwareCheck):
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
            HardwareCheck.print_error('Error opening: {} [baud {}]\t\t[FAIL]'
                .format(self.device, self.baud))
            return False
        dxl.servo_dev.setTimeout(0.1)
        result = True
        for i in self.ids:
            try:
                Robotis_Servo(dxl, i)
                HardwareCheck.print_ok('\tMotor ID: {}\t\t[OK]'.format(i))
            except:
                HardwareCheck.print_error('\tMotor ID: {} not found\t\t[FAIL]'
                    .format(i))
                result = False
        return result

def main():
    # Check l_arm
    l_arm = HardwareCheck()
    l_arm.add_child(FileCheck('/dev/bender/l_port'))
    l_arm.add_child(DynamixelCheck('/dev/bender/l_port', 200000,
        [1, 2, 3, 4, 5, 6, 7, 8]))
    l_arm_res = l_arm.check()
    # Check r_arm
    r_arm = HardwareCheck()
    r_arm.add_child(FileCheck('/dev/bender/r_port'))
    r_arm.add_child(DynamixelCheck('/dev/bender/r_port', 200000,
        [11, 12, 13, 14, 15, 16, 17, 18]))
    r_arm_res = r_arm.check()
    # Return
    HardwareCheck.exit(l_arm_res and r_arm_res)

if __name__ == "__main__":
    main()
