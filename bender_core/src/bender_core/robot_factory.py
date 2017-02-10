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

from core.light_head import LightHead

from core.arm     import LeftArmSkill, RightArmSkill
from core.gripper import LeftGripperSkill, RightGripperSkill

# str to Skill class dict
_str_to_skill = {
    LightHead._type : LightHead,
    KnowledgeSkill._type : KnowledgeSkill,
    RGBDSkill._type : RGBDSkill,
    LaserSkill._type : LaserSkill,
    JoySkill._type : JoySkill,
    HeadSkill._type : HeadSkill,
    BaseSkill._type : BaseSkill,
    SoundSkill._type : SoundSkill,
    TTSSkill._type : TTSSkill,
    LeftArmSkill._type : LeftArmSkill,
    RightArmSkill._type : RightArmSkill,
    LeftGripperSkill._type : LeftGripperSkill,
    RightGripperSkill._type : RightGripperSkill
}

def build(skills=_str_to_skill.keys()):
    """
    Build a robot object based on a skill list. By default build
    a robot using all core skills.

    Args:
        skills (list of str): Skill list.

    Raises:
        TypeError: If `skills` is not a list.
    """
    rospy.loginfo("factory: building robot ... ")
    robot = Robot("bender")

    # Check arg
    if not isinstance(skills, list):
        raise TypeError("skills must be a string list")

    # Add skill instance to robot    
    for skill_name in skills:
        if skill_name in _str_to_skill:
            robot.set(_str_to_skill[skill_name].get_instance())

            # shortcuts
            if skill_name == 'tts':
                robot.say = robot.tts.say
            
        else:
            rospy.logerr("Skill '{0}' is not registered".format(skill_name))

    rospy.loginfo("factory: the robot is built")
    return robot

