#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import sys

# Octomap
# TODO: eliminar estas dependencias del brazo!!
from bender_arm_planning.srv import ManageOctomap, ManageOctomapRequest
from bender_arm_planning.msg import OctomapOptions

from bender_utils.ros import benpy

class OctomapManager():
    """Manejo de octomap"""
    def __init__(self):
        # Obtener servicio
        self.octomap = benpy.ServiceProxy('/manage_octomap', ManageOctomap)

    def update(self):
        octomap_opt = OctomapOptions(OctomapOptions.UPDATE)
        self.octomap(octomap_opt)

    def stop(self):
        octomap_opt = OctomapOptions(OctomapOptions.STOP)
        self.octomap(octomap_opt)

    def start(self):
        octomap_opt = OctomapOptions(OctomapOptions.START)
        self.octomap(octomap_opt)

    def clear(self):
        octomap_opt = OctomapOptions(OctomapOptions.CLEAR)
        self.octomap(octomap_opt)

