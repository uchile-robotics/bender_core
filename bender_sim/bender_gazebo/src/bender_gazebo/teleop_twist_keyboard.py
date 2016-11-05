#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Twist message publisher from keyboard
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rmunozriffo@ing.uchile.cl'

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time
import numpy as np
from threading import Thread


class RingBuffer():
    """A 1D ring buffer using numpy arrays"""
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0

    def add(self, x):
        """Adds x to ring buffer"""
        self.data[self.index] = x
        self.index = (self.index + 1) % self.data.size

    def extend(self, x):
        """Adds array x to ring buffer"""
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        """Returns the first-in-first-out data in the ring buffer"""
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        return self.data[idx]

    def mean(self):
        """Returns the mean of buffer"""
        return np.mean(self.data)
     

class KeyboardTwist(object):
    """Reading from the keyboard"""
    def __init__(self, timeout=0.05, buffer_len=10, key_bindings=None):
        if key_bindings:
            self.bindings = key_bindings
        else:
            # Default value for key bindings usign W A S D
            self.bindings = self.bindings = {
                'w':(1,0),
                'a':(0,1),
                's':(-1,0),
                'd':(0,-1)
            }
        self.tty_settings = termios.tcgetattr(sys.stdin)
        self.timeout = timeout
        self.linear_buf = RingBuffer(buffer_len)
        self.angular_buf = RingBuffer(buffer_len)
        self.running = True
        Thread(target=self.update).start()

    def get_key(self):
        """Get pressed key"""
        tty.setraw(sys.stdin.fileno())
        key = None
        readable, writable, exceptional = select.select([sys.stdin], [], [], self.timeout)
        if readable and readable[0]==sys.stdin:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_settings)
        return key

    def update(self):
        """Thread main loop"""
        while self.running:
            key = self.get_key()
            linear = 0.0
            angular = 0.0
            # Capture Ctrl + C
            if key == '\x03':
                self.running = False
                break
            if key in self.bindings:
                linear = self.bindings[key][0]
                angular = self.bindings[key][1]
            self.linear_buf.add(linear)
            self.angular_buf.add(angular)


    def get_twist(self, twist):
        """Update Twist using data from buffers"""
        if not self.running:
            raise rospy.ROSInterruptException('key interrupt')
        twist.linear.x = self.linear_buf.mean()
        twist.angular.z = self.angular_buf.mean()

    def close(self):
        """Close main thread and restore stdin"""
        self.running = False
        time.sleep(self.timeout)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_settings)

def main():
    rospy.init_node('twist_keyboard_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    rate = rospy.Rate(20.0)
    key = KeyboardTwist()
    try:
        while not rospy.is_shutdown():
            key.get_twist(twist)
            pub.publish(twist)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Send empty message
        pub.publish(Twist())
        key.close()

if __name__ == '__main__':
    main()
