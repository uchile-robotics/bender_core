#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Gonzalo Olave, Rodrigo Muñoz'

from trajectory_msgs.msg import JointTrajectory
from os import listdir
import rospy
import rospkg
import yaml
import genpy

class CheckpointPlanner(object):

    def __init__(self):
        self.checkpoints = dict()
        rospack = rospkg.RosPack()
        traj_path = rospack.get_path('bender_arm')+'/config/predefined_trajectories/'
        traj = listdir(traj_path)

        for t in traj:
            # genera los checkpoints
            pos = t.split('.')
            # checkea si existe el nodo, si no, crea uno nuevo
            if pos[0] not in self.checkpoints.keys():
                self.checkpoints[pos[0]] = Node(pos[0])
            traj_data = None
            try:
                with open(traj_path+t, 'r') as f:
                    # load all documents
                    traj_data = yaml.load(f)
                    if traj_data is None:
                        raise yaml.YAMLError("Empty files not allowed")

                    # rellena los checkpoints con las trajectorias predefinidas
                    # print traj_data['header']
                    traj = JointTrajectory()
                    genpy.message.fill_message_args(traj,traj_data)
                    self.checkpoints[pos[0]].add_trayectory(pos[1],traj)

            except yaml.YAMLError as e:
                rospy.logerr('Invalid YAML file: %s' % (str(e)))

        #print self.checkpoints['pre_1'].reach.keys()

    def go(self, init, goal):
        # Obtener checkpoint mas cercano @TODO
        #current = self.get_current_state()
        
        # print self.checkpoints[init].reach[goal]
        # Check si goal es valido
        if init in self.checkpoints:
            if goal in self.checkpoints[init].reach:
                return self.checkpoints[init].reach[goal]
            else:
                return None
        else:
            return None

    def get_current_state(self):
        # @TODO
        return 'home'


class Node(object):

    def __init__(self,name = ''):
        self.name = name
        self.reach = dict()
      
    def add_trayectory(self, name, traj):
        self.reach[name]=traj


if __name__ == '__main__':

    rospy.init_node('checkpoint_planner_test')

    CHK = CheckpointPlanner()
    # print CHK.go('home','pre_1')
    print CHK.go('carry','shelf_3')
