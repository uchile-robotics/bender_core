#!/usr/bin/env python

from threading import Thread, Lock
from geometry_msgs.msg import Twist
import rospy

class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        
        # ros interface
        self.pub = rospy.Publisher('output', Twist, queue_size=5)
        self.sub = rospy.Subscriber('input', Twist, self.input_cb)

        # clock
        self.rate_pub = rospy.Rate(10)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()

        # thread for publishing stuff
        self.msg_lock = Lock()
        self.thread = Thread(target=self.publish_state)
        self.thread.start()
    
    def input_cb(self, msg):
        self.last_msg_time = rospy.Time.now()
        with self.msg_lock:
            self.last_msg = msg

    def publish_state(self):
        while not rospy.is_shutdown():
            if rospy.Time.now() - self.last_msg_time > rospy.Duration(2):
                self.last_msg.linear.x = 0.0
                self.last_msg.angular.z = 0.0

            with self.msg_lock:
                self.pub.publish(self.last_msg)
            self.rate_pub.sleep()


def main():
    rospy.init_node('cmd_vel_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
