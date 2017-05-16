#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import math
# ROS
import rospy
# Msgs
from std_msgs.msg import Float64
# Dynamic reconfigure
from dynamic_reconfigure.server import Server as DynamicReconfServer
from control_util.cfg import SourceConfig

class CommandSource():
    def __init__(self):
        self.topic_name = 'command'
        self.pub = rospy.Publisher(self.topic_name, Float64, queue_size = 20)
        # Dynamic reconfigure server, load dinamic parameters
        self.reconfig_server = DynamicReconfServer(SourceConfig, self.update_params)
        
    def update_params(self, config, level):
        # Update params
        self.frec = config.frec
        self.offset = config.offset
        self.amplitude = config.amplitude
        self.type = config.type
        if(self.topic_name != config.topic):
            rospy.loginfo("New config topic {}".format(config.topic))
            self.topic_name = config.topic
            self.pub = rospy.Publisher(self.topic_name, Float64, queue_size = 20)
        return config

    def get(self):
        if self.type == 0:
            return self.sine()
        elif self.type == 1:
            return self.square()

    def sine(self):
        return self.amplitude*math.sin(2*math.pi*self.frec*rospy.Time.now().to_sec())+self.offset

    def square(self):
        if math.sin(2*math.pi*self.frec*rospy.Time.now().to_sec()) > 0:
            return self.amplitude+self.offset
        else:
            return -self.amplitude+self.offset

    def publish(self):
        self.pub.publish(data=self.get())

def main():
    rospy.init_node('command_source')
    # Command publisher
    command = CommandSource()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        command.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
