#! /usr/bin/env python
## %Tag(FULLTEXT)%

import rospy
import math


class Object():
    def __init__(self, _name, _pose, _bboxes):
        self.name = _name # object name
        self.posestamped = _pose # center pose
        self.bboxes = _bboxes # bboxes 2d
        self.color = "unknown" # dominant color, options: red, blue, yellow, green, black, white
        self.form = "unknown" # form, options : box, sphere, cylinder
        self.hight = 0 # hight, information 3d 
        self.width = 0 # width, information 3d 
        self.long = 0 # long, information 3d 


class Face():
    def __init__(self, _pose, _bboxes):
        self.name = "unknown" # object name
        self.pose = _pose # center pose
        self.bboxes = _bboxes # bboxes 2d
        self.emotion = "unknown" # dominant emotion, options: sad, happy, angry...
