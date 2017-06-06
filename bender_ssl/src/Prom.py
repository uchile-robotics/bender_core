#!/usr/bin/env python

import rospy
import numpy as n
from std_msgs.msg import Float32
from hark_msgs.msg import HarkSource, HarkSourceVal, promxy

class MicroconeMark(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/hark_source', HarkSource, self.prom_callback)
        self.pub = rospy.Publisher('/prom_topic', promxy, queue_size=10)
        self.medicionesx = []
        self.promx = Float32()
        self.medicionesy = []
        self.promy = Float32()
        self.prom = promxy()

    def prom_callback(self, msg):
        if len(self.medicionesx) == 10 and len(self.medicionesy) == 10:
            self.promx = self.calcProm(self.medicionesx)
            self.promy = self.calcProm(self.medicionesy)
            self.prom.x = self.promx
            self.prom.y = self.promy
            self.pub.publish(self.prom)
            self.medicionesx = []
            self.medicionesy = []
        if msg.src:
            self.medicionesx.append(msg.src[0].x)
            self.medicionesy.append(msg.src[0].y)
        if not msg.src:
            self.medicionesx = []
            self.medicionesy = []

    def calcProm(self, x):
        dv_permitido = 5.0
        if len(x) == 10:
            data = x
            prom=n.mean(data)
            dataNew = []
            for i in data:
                if prom+dv_permitido > i and i > prom-dv_permitido:
                    dataNew.append(i)
            data = dataNew
            prom=n.mean(data)
        return prom

def main():
    rospy.init_node('base_controller3')
    rospy.loginfo('Init base controller2')
    base = MicroconeMark()   
    rospy.spin()

if __name__ == '__main__':
    main()