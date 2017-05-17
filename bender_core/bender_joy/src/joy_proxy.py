#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
from sensor_msgs.msg import Joy
from bender_joy import xbox


class JoystickProxy(object):
  
    def __init__(self):
        rospy.loginfo('Joystick proxy init ...')
    
        # control
        # default: active on channel 0
        self.current_channel_name = 'base'
        
        # load params
        self.proxy_button    = rospy.get_param('~b_proxy', 'BACK')
        self.channels        = rospy.get_param('~channels', { 'base': {'button': 'A', 'topic': 'base'}})
        self.default_channel = rospy.get_param('~default_channel', 'base')
        
        # check params
        if not self.params_are_valid():
            sys.exit(1)

        # map ids and map button names to button ids
        key_mapper = xbox.KeyMapper()
        self.proxy_button_id = key_mapper.get_button_id(self.proxy_button)
        for channel_name, channel in self.channels.items():
            button = self.channels[channel_name]['button']
            topic  = self.channels[channel_name]['topic']
            self.channels[channel_name]["id"] = key_mapper.get_button_id(button)
            self.channels[channel_name]["publisher"] = rospy.Publisher(topic, Joy, queue_size=10)
        
        # enable channel
        self.enable_channel(self.default_channel)

        # ready to work
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.loginfo('Joystick proxy is ready')
        

    def params_are_valid(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.proxy_button, str)
        assert isinstance(self.default_channel, str)
        assert isinstance(self.channels, dict)

        button_set = set()

        key_mapper = xbox.KeyMapper()
        if not key_mapper.has_button(self.proxy_button):
            rospy.logerr("proxy: unknown button '%s'" % self.proxy_button)
            return False
        button_set.add(self.proxy_button)

        for channel_name, channel in self.channels.items():
            assert isinstance(channel_name, str)
            assert isinstance(channel, dict)
            if 'button' not in channel:
                rospy.logerr("proxy: channel '%s' does not has 'button' field" % channel_name)
                return False
            if 'topic' not in channel:
                rospy.logerr("proxy: channel '%s' does not has 'topic' field" % channel_name)
                return False

            assert isinstance(channel['button'], str)
            assert isinstance(channel['topic'], str)
            if not key_mapper.has_button(channel['button']):
                rospy.logerr("proxy: button '%s' is invalid for channel '%s'" % (channel['button'], channel_name))
                return False

            if channel['button'] == self.proxy_button:
                rospy.logerr("proxy: button for channel '%s' cannot be equals to the proxy button" % channel_name)
                return False

            button_set.add(channel['button'])

        if self.default_channel not in self.channels:
            rospy.logerr("proxy: default channel '%s' is not into the available options" % self.default_channel)
            return False
        
        if len(button_set) != len(self.channels) + 1:
            rospy.logerr("proxy: all buttons must be different!")
            return False

        return True
        

    def forward(self, msg):
        """
        forwards Joy messages to the configured topic
        """
        self.channels[self.current_channel_name]['publisher'].publish(msg)


    def enable_channel(self, channel_name):
        """
        """
        if self.current_channel_name == channel_name:
            rospy.logwarn("proxy: is already configured to this channel '%s'" % channel_name)
            return
        self.current_channel_name = channel_name
        rospy.logwarn("proxy: configured joy to channel named '%s'" % channel_name)


    def parse_channel_name(self, msg):
        """
        parses a Joy msg for proxy actions
        returns the selected channel on success
        returns None otherwise
        """
        # list queried channels
        queried_channels = []
        for channel_name, channel in self.channels.items():
            if msg.buttons[channel['id']]:
                queried_channels.append(channel_name)

        # no channel
        if len(queried_channels) == 0:
            # the is no action to be done
            return None

        # too many channels
        if len(queried_channels) > 1:
            rospy.logwarn("proxy: you cannot set more than one configurations at a time!.")
            return None

        return queried_channels[0]


    def callback(self, msg):
        
        # forward if msg is not proxy related
        if not msg.buttons[self.proxy_button_id]:
            self.forward(msg)
            return

        # enable selected channel
        channel_name = self.parse_channel_name(msg)
        if channel_name is not None:
            self.enable_channel(channel_name)
        return
        

if __name__ == '__main__':
    rospy.init_node('joy_proxy')
    JoystickProxy()
    rospy.spin()
