#!/usr/bin/env python
from __future__ import print_function
import argparse
import os
from lib_robotis import *

class colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Check Dynamixel motors
def dxl_check(device,baud,ids):
    try:
        dxl = USB2Dynamixel_Device(device, baud)
    except:
        print(colors.FAIL + 'Error opening: {} [baud {}]\t\t[FAIL]'.format(device,baud) + colors.END)
        return

    dxl.servo_dev.setTimeout(0.1)
    for i in ids:
        try:
            Robotis_Servo(dxl, i)
            print(colors.OKGREEN + '\tMotor ID: {}\t\t[OK]'.format(i) + colors.END)
        except:
            print(colors.FAIL + '\tMotor ID: {} not found\t\t[FAIL]'.format(i) + colors.END)
      
    dxl.servo_dev.setTimeout( 1.0 ) # Restore



def hw_check():
    # Check l_arm
    if os.path.exists('/dev/bender/left_arm'):
        print(colors.OKGREEN + 'USB2Dynamixel l_arm \t\t[OK]' + colors.END)
        dxl_check('/dev/bender/left_arm', 200000, [1,2,3,4,5,6,7,8])
    else:
        print(colors.FAIL + 'USB2Dynamixel l_arm not found\t\t[FAIL]' + colors.END)

    # Check r_arm
    if os.path.exists('/dev/bender/right_arm'):
        print(colors.OKGREEN + 'USB2Dynamixel r_arm \t\t[OK]' + colors.END)
        dxl_check('/dev/bender/right_arm', 200000, [11,12,13,14,15,16,17,18])
    else:
        print(colors.FAIL + 'USB2Dynamixel r_arm not found\t\t[FAIL]' + colors.END)

if __name__ == "__main__":
    hw_check()