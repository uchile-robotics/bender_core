#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
from sensor_msgs.msg import Joy

# TODO: n channels
# TODO: default
#    force a default channel at start
#    default_channel: 0
# TODO: [WARN] [/joy/proxy0]: Inbound TCP/IP connection failed: connection
# from sender terminated before handshake header received. 0 bytes were 
# received. Please check sender for additional details.
# TODO: decir el tópico, nombre de config y botón que quedó configurado.

class JoystickProxy(object):
  
    def __init__(self):
        rospy.loginfo('Joystick proxy init ...')
        rospy.Subscriber('joy', Joy, self.callback)
    
        # control
        # default: active on channel 0
        self.is_active = True
        self.current_channel_id = 0
        
        # load configuration
        self.channel_button_ids = rospy.get_param('~channel_button_ids', [0, 1, 2, 3])
        self.proxy_button_id    = rospy.get_param('~proxy_button_id', 8)
        self.disable_button_id  = rospy.get_param('~disable_button_id', 6)
        self.publishers = [
            rospy.Publisher('topic_A', Joy, queue_size=10),
            rospy.Publisher('topic_B', Joy, queue_size=10),
            rospy.Publisher('topic_X', Joy, queue_size=10),
            rospy.Publisher('topic_Y', Joy, queue_size=10)
        ]

        # check
        if not self.params_are_valid():
            sys.exit(1)

        # enable channel
        self.enable_channel(self.current_channel_id)


        self.max_id = max(self.channel_button_ids)
        self.max_id = max([self.max_id, self.proxy_button_id, self.disable_button_id])

        rospy.loginfo('Joystick proxy is ready')


    def params_are_valid(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.channel_button_ids, list)
        assert isinstance(self.proxy_button_id,    int)
        assert isinstance(self.disable_button_id,  int)

        are_valid = True
        for button_id in self.channel_button_ids:
            if button_id < 0:
                rospy.logerr("proxy: button index must be greater o equal to zero (passed: %d)" % button_id)
                are_valid = False

        n_buttons = len(self.channel_button_ids)
        if n_buttons != 4:
            rospy.logerr("proxy: you must provide 4 configuration buttons (found: %d)" % n_buttons)
            are_valid = False

        button_set = set(self.channel_button_ids)
        button_set.add(self.proxy_button_id)
        button_set.add(self.disable_button_id)
        if len(button_set) != 6:
            rospy.logerr("proxy: all buttons must be different!")
            are_valid = False
        return are_valid
        

    def forward(self, msg):
        """
        forwards Joy messages to the configured topic
        """
        if self.is_active:
            pub = self.publishers[self.current_channel_id]
            pub.publish(msg)


    def disable_forwarding(self):
        """
        disables joy forwarding
        """
        if self.is_active:
            rospy.logwarn("proxy: control disabled")
            self.is_active = False


    def enable_forwarding(self):
        """
        enables joy forwarding
        """
        if not self.is_active:
            rospy.logwarn("proxy: control enabled")
            self.is_active = True


    def enable_channel(self, channel_id):
        """
        """
        self.enable_forwarding()

        # set configuration
        if self.current_channel_id == channel_id:
            rospy.logwarn("proxy: is already configured to the channel_id %d" % self.current_channel_id)
            return
        self.current_channel_id = channel_id
        rospy.logwarn("proxy: configured joy to the channel_id %d" % channel_id)


    def parse_channel_id(self, msg):
        """
        parses a Joy msg for proxy actions
        returns the selected channel on success
        returns None otherwise
        """
        # list queried channels
        queried_channel_ids = []
        for channel_id, button_id in enumerate(self.channel_button_ids):
            if msg.buttons[button_id]:
                queried_channel_ids.append(channel_id)

        # no channel
        if len(queried_channel_ids) == 0:
            # the is no action to be done
            return None

        # too many channels
        if len(queried_channel_ids) > 1:
            rospy.logwarn("proxy: you cannot set two configurations at a time!.")
            return None

        return queried_channel_ids[0]


    def callback(self, msg):
        
        # check max value        
        max_available_id = len(msg.buttons) - 1
        if self.max_id > max_available_id:
            rospy.logwarn("proxy: button index (%d) is greater than the max permitted value (%d)" % (self.max_id, max_available_id))
            return

        # forward if msg is not proxy related
        if not msg.buttons[self.proxy_button_id]:
            self.forward(msg)
            return

        # disable joy
        if msg.buttons[self.disable_button_id]:
            self.disable_forwarding()
            return

        # enable selected channel
        channel_id = self.parse_channel_id(msg)
        if channel_id is not None:
            self.enable_channel(channel_id)
        return
        

if __name__ == '__main__':
    rospy.init_node('joy_proxy')
    JoystickProxy()
    rospy.spin()
