#!/usr/bin/python
__author__ = 'Gustavo Diaz, Rodrigo Munoz'

import math
import rospy

from threading import Thread
from std_msgs.msg import Bool

class HeadController:

    LED_ADDRESS = 6

    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.state = Bool()
        
    def initialize(self):
        # Get params and allocate msgs
        # Get rate parameter
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/device/rate', 2)
        rospy.loginfo("head_controller at rate {}".format(self.state_update_rate))
        # Get id parameter (required)
        if not rospy.has_param(self.controller_namespace + '/device/id'):
            # Return error
            rospy.logerr('Parameter {} does not exist'.format(rospy.get_namespace() + self.controller_namespace + '/device/id'))
            return False
        self.id = rospy.get_param(self.controller_namespace + '/device/id')
        rospy.loginfo("head_controller device ID {}".format(self.id))
        
        self.state.data = False
        return True

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Bool, self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', Bool, queue_size = 5)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_command(self, msg):
        data = 0
        if msg.data:
            data = 1
        try:
            result = self.dxl_io.write(self.id, HeadController.LED_ADDRESS,[data])
        except Exception as ex:
            rospy.logerr('Exception thrown while writing addres {}'.format(HeadController.LED_ADDRESS))

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            self.state_pub.publish(self.state)
            rate.sleep()

