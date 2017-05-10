#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
from bender_utils.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from bender_fieldbus.check import DynamixelCheck

def r_arm_check():
    r_arm = SystemCheck("right arm")
    r_arm.add_child(FileCheckTask('/dev/bender/r_port'))
    r_arm.add_child(DynamixelCheck('/dev/bender/r_port', 200000,
        [11, 12, 13, 14, 15, 16, 17, 18]))
    return r_arm.check()

def l_arm_check():
    l_arm = SystemCheck("left arm")
    l_arm.add_child(FileCheckTask('/dev/bender/l_port'))
    l_arm.add_child(DynamixelCheck('/dev/bender/l_port', 200000,
        [1, 2, 3, 4, 5, 6, 7, 8]))
    return l_arm.check()
