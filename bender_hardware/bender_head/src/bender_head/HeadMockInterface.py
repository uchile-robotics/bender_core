#!/usr/bin/env python
# Matias Pavez Bahamondes


import rospy

from bender_head.AbstractHeadInterface import AbstractHeadInterface


class HeadMockInterface(AbstractHeadInterface):
    ''' mocks the communication with a real bender-head. '''

    def __init__(self, msg_keys):            
        self._is_connected_ = False
        self.msg_keys = msg_keys

    def connect(self):
        self._is_connected_ = True
        rospy.logwarn("connected to Mocked head hardware")
        return True

    def close(self):
        self._is_connected_ = False
        rospy.logwarn("disconnected from Mocked head hardware")
        return True

    def is_connected(self):
        return self._is_connected_
        

    def write(self, key, value):
        if key not in self.msg_keys:
            rospy.logwarn("unknown key: %s" % key)
            return False
        rospy.logwarn("writting to a mocked Head: (%s, '%s')" % (key, value) )
        return True


    def loop(self):
        rospy.sleep(0.5)
        return True

