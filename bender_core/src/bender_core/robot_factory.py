#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
__author__ = 'Matías Pavez'
__email__  = 'matias.pavez@ing.uchile.cl'

import rospy
from robot import Robot

## CORE imports
# knowledge base
from core.knowledge import KnowledgeSkill

# sensors
from core.rgbd import RGBDSkill
from core.laser import LaserSkill

# hardware
from core.joy import JoySkill
from core.head import HeadSkill
from core.base import BaseSkill
from core.sound import SoundSkill
from core.tts import TTSSkill

from core.arm     import LeftArmSkill, RightArmSkill
from core.gripper import LeftGripperSkill, RightGripperSkill


def build():

    rospy.loginfo(" building robot ... ")
    robot = Robot("bender")

    # knowledge base
    robot.set(KnowledgeSkill.get_instance())

    # hardware
    robot.set(LeftArmSkill.get_instance())
    robot.set(RightArmSkill.get_instance())
    robot.set(LeftGripperSkill.get_instance())
    robot.set(RightGripperSkill.get_instance())

    robot.set(BaseSkill.get_instance())
    robot.set(HeadSkill.get_instance())
    robot.set(SoundSkill.get_instance())
    robot.set(TTSSkill.get_instance())
    robot.set(JoySkill.get_instance())

    # sensors
    robot.set(RGBDSkill.get_instance())
    robot.set(LaserSkill.get_instance())

    rospy.loginfo(" ... ready")
    return robot

