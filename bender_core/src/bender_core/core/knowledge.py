#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot_skill import RobotSkill
from bender_core.robot_skill import Log

from bender_msgs.msg import SemanticObject as ps_msg
from bender_srvs.srv import SemMap as ps_srv
from bender_srvs.srv import String as string_srv


class KnowledgeSkill(RobotSkill):
    """
    The KnowledgeSkill

    Provides an interface to the robot knowledge metapackage

    Currently the only implemented nodes are:
    - pose_server
    """
    _type = "knowledge"

    def __init__(self):
        super(KnowledgeSkill, self).__init__()
        self._description = "the knowledge skill"
        self.pose = PoseServerWrapper()
    
    def setup(self):
        self.pose.setup()
        return True

    def check(self):
        self.pose.check()
        return True

    def start(self):
        self.pose.start()
        return True

    def pause(self):
        self.pose.pause()
        return True

    def shutdown(self):
        self.pose.shutdown()
        return True


class PoseServerWrapper(object):
    """
    docstring for PoseServerWrapper

    TODO FUTURO: USO ESTILO DICCIONARIO PYTHON>  pose['living room'] = Point...
    """
    
    def __init__(self):
        self.which_map_client = rospy.ServiceProxy('/bender/knowledge/pose_server/which', string_srv)
        self.load_map_client  = rospy.ServiceProxy('/bender/knowledge/pose_server/load', string_srv)
        self.delete_client    = rospy.ServiceProxy('/bender/knowledge/pose_server/delete', ps_srv)
        self.get_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/get', ps_srv)
        self.get_all_client   = rospy.ServiceProxy('/bender/knowledge/pose_server/get_all', ps_srv)
        self.print_client     = rospy.ServiceProxy('/bender/knowledge/pose_server/print', ps_srv)
        self.save_client      = rospy.ServiceProxy('/bender/knowledge/pose_server/save', ps_srv)
        self.set_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/set', ps_srv)
        

    def setup(self):
        self.which_map_client = rospy.ServiceProxy('/bender/knowledge/pose_server/which', string_srv)
        self.load_map_client  = rospy.ServiceProxy('/bender/knowledge/pose_server/load', string_srv)
        self.delete_client    = rospy.ServiceProxy('/bender/knowledge/pose_server/delete', ps_srv)
        self.get_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/get', ps_srv)
        self.get_all_client   = rospy.ServiceProxy('/bender/knowledge/pose_server/get_all', ps_srv)
        self.print_client     = rospy.ServiceProxy('/bender/knowledge/pose_server/print', ps_srv)
        self.save_client      = rospy.ServiceProxy('/bender/knowledge/pose_server/save', ps_srv)
        self.set_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/set', ps_srv)
        return True

    def check(self):
        try:
            self.which_map_client.wait_for_service(timeout=2)
            self.load_map_client.wait_for_service(timeout=2)
            self.delete_client.wait_for_service(timeout=2)
            self.get_client.wait_for_service(timeout=2)
            self.get_all_client.wait_for_service(timeout=2)
            self.print_client.wait_for_service(timeout=2)
            self.save_client.wait_for_service(timeout=2)
            self.set_client.wait_for_service(timeout=2)
        except rospy.ROSException:
            rospy.logwarn('[knowledge.pose]: Setup failed. PoseServer services not found. e.g. /bender/knowledge/pose_server/which')
            return False
        return True

    def start(self):
        return True

    def pause(self):
        return True

    def shutdown(self):
        return True

    def keys():
        return []


