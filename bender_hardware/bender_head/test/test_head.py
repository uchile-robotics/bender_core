#!/usr/bin/env python

####################################
##   This file shows the basic    ##
## usage of the node 'head.py'.   ##
####################################

# R O S
import roslib
roslib.load_manifest('bender_head')
import rospy

# P y t h o n 
import sys
import math
import time

# Messages
from std_msgs.msg import Bool
from bender_msgs.msg import Emotion

rospy.init_node("test_head")
head  = rospy.Publisher('/bender/hw/head/cmd', Emotion, queue_size=10)
mouth = rospy.Publisher('/bender/hw/head/move_mouth', Bool, queue_size=10)

print "Presione Enter"
signal = raw_input()


print "Talking ... "
mouth.publish(True)
time.sleep(3)
print "Stopping talk ... "
mouth.publish(False)
time.sleep(3)


print "Yaw: 10 ..."
head.publish("MoveX", "happy1", 10)

print "Emotion: happy1 ... "
head.publish("changeFace", "happy1", 0)
time.sleep(3)

print "Emotion: happy2 ... "
head.publish("changeFace", "happy2", 0)
time.sleep(4)

print "Emotion: happy3   and  Yaw: -20"
head.publish("MoveX", "happy1", -20)
head.publish("changeFace", "happy3", 0)
time.sleep(5)

print "Yaw: -20 ..."
head.publish("MoveX", "happy1", -20)
time.sleep(5)

print "Yaw: -10 ..."
head.publish("MoveX", "happy1", -10)
time.sleep(3)

print "Emotion: sad1 ... "
head.publish("changeFace", "sad1", 0)
time.sleep(3)

print "Emotion: sad2 ... "
head.publish("changeFace", "sad2", 0)
time.sleep(3)

print "Emotion: sad3 ... "
head.publish("changeFace", "sad3", 0)
time.sleep(3)

print "Emotion: angry1 ... "
head.publish("changeFace", "angry1", 0)
time.sleep(3)

print "Emotion: angry2 ... "
head.publish("changeFace", "angry2", 0)

print "Emotion: angry3 ... "
head.publish("changeFace", "angry3", 0)
time.sleep(5)

print "Emotion: serious ... "
head.publish("changeFace", "serious", 0)
time.sleep(3)

print "Emotion: surprise ... "
head.publish("changeFace", "surprise", 0)
time.sleep(3)

print "Emotion: eyebrow ... "
head.publish("changeFace", "eyebrow", 0)
time.sleep(3)

print "Emotion: flirt    and  yaw: -20 "
head.publish("changeFace", "flirt", 0)
head.publish("MoveX", "happy1", -20)
time.sleep(3)

print "Emotion: happy3 ... "
head.publish("changeFace", "happy3", 0)
time.sleep(6)

print "Emotion: serious  and Yaw: 0"
head.publish("MoveX", "happy1", 0)
head.publish("changeFace", "serious", 0)

print "T H E    E N D  :D!!!"
