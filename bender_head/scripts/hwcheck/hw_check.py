#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from bender_fieldbus.check import DynamixelCheck

def head_check():
    head = SystemCheck("head")
    head.add_child(FileCheckTask('/dev/bender/r_port'))
    head.add_child(DynamixelCheck('/dev/bender/r_port', 200000, [30,31,35]))
    return head.check()

if __name__ == "__main__":
    if head_check():
        sys.exit(0)
    sys.exit(1)
