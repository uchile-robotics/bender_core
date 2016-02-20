#!/usr/bin/env python

'''
Bender Gripper Action Client
'''
import sys

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)


class GripperClient(object):
    def __init__(self, gripper):
        ns = '/bender/' + gripper + '_controller/'
        rospy.loginfo('Init GripperClient con %s' % ns + 'gripper_action')
        self._client = actionlib.SimpleActionClient(ns + 'gripper_action', GripperCommandAction)
        self._goal = GripperCommandGoal()
        rospy.loginfo('Waiting for server...')
        conection = self._client.wait_for_server()
        print conection
        # Wait 10 Seconds for the gripper action server to start or exit
        if not conection:
            rospy.logerr('Exiting - %s Gripper Action Server Not Found' % gripper)
            rospy.signal_shutdown('Action Server not found')
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        rospy.loginfo('Sending new goal with pos: %.2f eff: %.2f' % (position, effort))
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    '''Bender Gripper Action Client

    Client of the Gripper Action Server for sending commands of standard action type
    control_msgs/GripperCommand.
    '''
    rospy.init_node('gripper_action_client')

    gc = GripperClient('r_gripper')
    gc.command(position=0.1, effort=10.0)
    gc.wait()
    #gc.command(position=1.0, effort=1.0)
    #gc.wait()
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    main()
