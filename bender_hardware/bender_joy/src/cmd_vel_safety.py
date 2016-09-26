#!/usr/bin/env python

from threading import Thread, Lock
from geometry_msgs.msg import Twist
import rospy

class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self, rate=60):
        # Publisher and Subscriber for cheking safety
        self.safe_pub = rospy.Publisher('/cmd_vel_safety_output', Twist, queue_size=5)
        self.safe_sub = rospy.Subscriber('/cmd_vel_safety_input', Twist, self.check_safety)

        # Rate, 60 Hz defaut
        self.rate_freq = rate
        self.rate_pub = rospy.Rate(self.rate_freq)
        # Fill message
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        # joystick input state
        self.joy_ts = rospy.Time.now()
        # Thread
        self.msg_lock = Lock()
        self.running = True
        self.thread = Thread(target=self.publish_state)
        self.thread.start()
    
    def check_safety(self, msg):
        self.joy_ts = rospy.Time.now()
        with self.msg_lock:
            self.msg = msg

    def publish_state(self):
        while self.running and not rospy.is_shutdown():
            if rospy.Time.now() - self.joy_ts > rospy.Duration(2):
                rospy.logwarn("JOYSTICK TIMEOUT, SENDING ZERO VELOCITY")
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0

            with self.msg_lock:
                self.safe_pub.publish(self.msg)
            self.rate_pub.sleep()

    def stop(self):
        self.running = False

def main():
    rospy.init_node('check_bender_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
