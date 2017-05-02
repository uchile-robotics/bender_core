#!/bin/sh

python $(rospack find bender_fieldbus)/src/bender_fieldbus/lib_robotis.py -d /dev/bender/left_arm --scan
