#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from bender_joy import xbox

from bender_skills import robot_factory

class BenderSpeaker(object):

    def __init__(self):

        # loading robot
        rospy.logwarn("Attemping to build Bender")
        self.robot      = robot_factory.build(["face","tts"],core=False)
        #self.neck       = self.robot.get("neck")
        #self.face       = self.robot.get("face")
        #self.l_arm      = self.robot.get("l_arm")
        #self.r_arm      = self.robot.get("r_arm")
        #self.l_gripper  = self.robot.get("l_gripper")
        #self.r_gripper  = self.robot.get("r_gripper")
        self.tts        = self.robot.get("tts")

        # tts config
        self.tts.set_language("spanish")
        self.tts.say("hola")
        self.tts.wait_until_done()
        self.tts.say("Hoy sere el anfitreon de esta presentacion")
        self.tts.wait_until_done()
        self.tts.say("Bienvenidos a Stark Expo")
        self.tts.wait_until_done()

        

if __name__ == '__main__':
    rospy.init_node('bender_speaker')
    bender = BenderSpeaker()
    #bender.talk()

    #rospy.spin()