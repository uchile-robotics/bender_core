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
    - pose_server (see below)

    Each robot_skill overriden method calls to the correspondent method from the "subskills"
    """
    _type = "knowledge"

    def __init__(self):
        super(KnowledgeSkill, self).__init__()
        self._description = "the knowledge skill"
        self.pose = PoseServerWrapper()
    
    def setup(self):
        return self.pose.setup()

    def check(self):
        return self.pose.check()

    def start(self):
        return self.pose.start()

    def pause(self):
        return self.pose.pause()

    def shutdown(self):
        return self.pose.shutdown()


class PoseServerWrapper(object):
    """
    Python interface to the Robot Pose Server node

    The pose server mantains a SemanticObject dictionary in memory, which can
    be dumped o loaded to/from a file, this way you can remember changes. 

    Loads the default map on start-up (bender_maps/maps/map.sem_map)
    Dumps to a backup map on shutdown (bender_maps/maps/<current_map>.bkp)

    TO BE DONE: DISPLAY THE SKILL OVERVIEW FOR THIS CLASS
    """
    
    def __init__(self):
        self.which_map_client = rospy.ServiceProxy('/bender/knowledge/pose_server/which', string_srv)
        self.load_map_client  = rospy.ServiceProxy('/bender/knowledge/pose_server/load', string_srv)
        self.save_client      = rospy.ServiceProxy('/bender/knowledge/pose_server/save', string_srv)
        self.delete_client    = rospy.ServiceProxy('/bender/knowledge/pose_server/delete', string_srv)
        self.get_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/get', ps_srv)
        self.get_all_client   = rospy.ServiceProxy('/bender/knowledge/pose_server/get_all', ps_srv)
        self.print_client     = rospy.ServiceProxy('/bender/knowledge/pose_server/print', ps_srv)
        self.set_client       = rospy.ServiceProxy('/bender/knowledge/pose_server/set', ps_srv)
        

    # =========================================================================
    # SKILLS INTERFACE
    # =========================================================================
    def setup(self):
        """
        The map is loaded by default with the map bender_maps/maps/map.sem_map
        """
        return True

    def check(self):
        """
        Checks for required services
        """
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
            rospy.logwarn('[knowledge.pose]: Check failed. PoseServer services not found. e.g. /bender/knowledge/pose_server/which')
            return False
        return True

    def start(self):
        """
        Nothing to do here
        """
        return True

    def pause(self):
        """
        Nothing to do here
        """
        return True

    def shutdown(self):
        """
        Dumps to a backup map on shutdown (bender_maps/maps/<current_map>.bkp)
        """
        current_map = self.map_name()
        self.save_to_map(current_map + ".bkp")
        return True
    # =========================================================================


    # =========================================================================
    # MAP METHODS
    # =========================================================================
    def get_default_map_name(self):
        return "map.sem_map"


    def map_name(self):
        """
        returns the current map name (String) on success (e.g: map.sem_map)
        returns None otherwise
        """
        try:
            res = self.which_map_client()
            return res.data
        except rospy.ServiceException:
            return None


    def new_map(self, map_name="new_map.sem_map"):
        """
        Creates an empty map
    
        returns True on success, False otherwise
        """
        keys = self.keys() 
        old_map = []
        for key in keys:
            old_map.append(self.get(key))

        # delete all
        if not self.delete_all():

            # recovery attempt
            for pose in old_map:
                self.set(pose)
            
            return False

        self.save_to_map(map_name)   # create new file
        self.load_from_map(map_name) # it is necessary to update the current map reference.
        return False


    def save_to_map(self, map_name=None):
        """
        Saves all current data on memory to a map file.

        The map_name file is completely overwritten by this operation.
        If the map_name is not provided, the current map file is used. 

        The current map entity is not modified, so future calls to map_name() will 
        return the same as the previous ones 

        returns True on success, False otherwise
        """
        if map_name is None:
            map_name = self.map_name()

        try:
            self.save_client(map_name)
            return True
        except rospy.ServiceException:
            return False


    def load_from_map(self, map_name=None):
        """
        Loads map data from a file. All previous data in memory is forgotten.

        If the map_name is not provided, the current map file is used. Which is
        useful to reset all current changes in memory.

        returns True on success, False otherwise
        """
        if map_name is None:
            map_name = self.map_name()

        try:
            self.load_map_client(map_name)
            return True
        except rospy.ServiceException:
            return False


    def load_default_map(self):
        """
        Loads the default map.

        see also: load_from_map(), get_default_map_name()
        """
        return self.load_from_map(self.get_default_map_name())


    # =========================================================================
    # POSE METHODS
    # =========================================================================

    def has(self, name):
        """
        returns True if there is a SemanticObject whose id is 'name'.
        returns None if the object is not found
        """
        keys = self._keys()
        if name in keys:
            return True
        return False


    def get(self, name):
        """
        returns the SemanticObject from memory whose id is 'name'.
        returns None if the object is not found
        """
        try:
            res = self.get_client(id=name)
            if res is not None and len(res.data) > 0:
                return res.data[0]
            return None
        except rospy.ServiceException:
            return None


    def set(self, semantic_object):
        """
        does not write to the file.. only to memory
        if you want to save the records, use save()
        """
        try:
            self.set_client(new_data=semantic_object)
        except rospy.ServiceException:
            return False
        return True


    def delete(self, name):
        """
        Deletes an object FROM MEMORY whose id is 'name'.

        If you want to preserve changes, use the save_to_map() method

        returns True on success, False otherwise
        """
        try:
            self.delete_client(name)
        except rospy.ServiceException:
            return False
        return True


    def delete_all(self):
        """
        Deletes all objects from map.

        returns True on success, False otherwise
        """
        keys = self._keys()
        for key in keys:
            if not self.delete(key):
                return False
        return True


    def keys(self):
        """
        returns A list of the known SemanticObject ids from memory.

        the list is empty if there was an error or the memory is empty.
        """
        keys = self._keys()
        return keys if keys is not None else []


    def location_keys(self):
        """
        The same as keys(), but restricted to 2D poses (which refers to locations)
        """
        keys = self._keys("map_pose")
        return keys if keys is not None else []


    def object_keys(self):
        """
        The same as keys(), but restricted to 3D poses (which refers to objects)
        """
        keys = self._keys("object_pose")
        return keys if keys is not None else []


    def _keys(self, posetype=None):
        """
        (private method!): Use this at your own risk.

        returns a list of the known SemanticObjects, whose type is 'posetype'.
        if posetype is None, then returns all keys.

        You can use this method, if you intend to save poses whose types differs from
        the ones for locations or objects.

        returns None on fail.
        """
        result = []
        try:
            res = self.get_all_client()
            for item in res.data:
                if posetype is not None:
                    if item.type == posetype:
                        result.append(item.id)    
                else:
                    result.append(item.id)
        except rospy.ServiceException:
            rospy.logwarn('[knowledge.pose]: keys() failed. "pose_server/get_all" service is alive?')
            return None
        return result