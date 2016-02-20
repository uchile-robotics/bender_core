#!/usr/bin/env python

####################################
##   This file shows the basic    ##
## usage of the node 'face.py'.   ##
####################################

# R O S
import roslib
roslib.load_manifest('bender_face')
import rospy

# P y t h o n 
import sys
import math
import time

# Messages
from bender_msgs.msg import Emotion

rospy.init_node("testFace")
face = rospy.Publisher('/bender/face/head',Emotion, queue_size=10)

print "Presione Enter"
signal = raw_input()

print "Yaw: 10 ..."
face.publish("MoveX", "happy1", 10)

print "Emotion: happy1 ... "
face.publish("changeFace", "happy1", 0)
time.sleep(3)

print "Emotion: happy2 ... "
face.publish("changeFace", "happy2", 0)
time.sleep(4)

print "Emotion: happy3   and  Yaw: -20"
face.publish("MoveX", "happy1", -20)
face.publish("changeFace", "happy3", 0)
time.sleep(5)

print "Yaw: -20 ..."
face.publish("MoveX", "happy1", -20)
time.sleep(5)

print "Yaw: -10 ..."
face.publish("MoveX", "happy1", -10)
time.sleep(3)

print "Emotion: sad1 ... "
face.publish("changeFace", "sad1", 0)
time.sleep(3)

print "Emotion: sad2 ... "
face.publish("changeFace", "sad2", 0)
time.sleep(3)

print "Emotion: sad3 ... "
face.publish("changeFace", "sad3", 0)
time.sleep(3)

print "Emotion: angry1 ... "
face.publish("changeFace", "angry1", 0)
time.sleep(3)

print "Emotion: angry2 ... "
face.publish("changeFace", "angry2", 0)

print "Emotion: angry3 ... "
face.publish("changeFace", "angry3", 0)
time.sleep(5)

print "Emotion: serious ... "
face.publish("changeFace", "serious", 0)
time.sleep(3)

print "Emotion: surprise ... "
face.publish("changeFace", "surprise", 0)
time.sleep(3)

print "Emotion: eyebrow ... "
face.publish("changeFace", "eyebrow", 0)
time.sleep(3)

print "Emotion: flirt    and  yaw: -20 "
face.publish("changeFace", "flirt", 0)
face.publish("MoveX", "happy1", -20)
time.sleep(3)

print "Emotion: happy3 ... "
face.publish("changeFace", "happy3", 0)
time.sleep(6)

print "Emotion: serious  and Yaw: 0"
face.publish("MoveX", "happy1", 0)
face.publish("changeFace", "serious", 0)

print "T H E    E N D  :D!!!"
