#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import sys
import rospy
from bender_core import robot_factory


"""
HOW TO RUN THIS EXAMPLE:

# to test the knowledge.pose skill
$ roslaunch bender_pose_server pose_server.launch
"""
if __name__ == "__main__":

    rospy.init_node("core_bot_example")

    print "--------------------------------------------------------------------"
    robot = robot_factory.build()
    print "--------------------------------------------------------------------"

    # check
    if not robot.knowledge.check():
        print "knowledge skill is not ready to start. BYE"
        sys.exit(1)

    # get all keys
    print "\nWhich poses do the robot know?: " 
    print robot.knowledge.pose.keys()


    # get only location/object keys
    print "\nWhich location poses do the robot know?: " 
    print robot.knowledge.pose.location_keys()
    print "\nWhich object poses do the robot know?: " 
    print robot.knowledge.pose.object_keys()
    

    # has key
    print "\nRobot knows the kitchen location?: " 
    print robot.knowledge.pose.has('kitchen')
    print "\nRobot knows the MADAFACA location?: " 
    print robot.knowledge.pose.has('MADAFACA')


    # get object (valid)
    print "\nTell me about the kitchen: " 
    print robot.knowledge.pose.get('kitchen')


    # get object (invalid)
    print "\nTell me about MADAFACA: " 
    print robot.knowledge.pose.get('MADAFACA')


    # set object (known)
    print "\nPlease, update the kitchen position: " 
    sample_pose = robot.knowledge.pose.get('kitchen')
    sample_pose.pose.position.x += 0.5
    robot.knowledge.pose.set(sample_pose)
    print robot.knowledge.pose.keys()


    # set object (unknown)
    print "\nPlease, record this new position: " 
    sample_pose.id = "somewhere-near-the-kitchen"
    sample_pose.pose.position.x += 1
    robot.knowledge.pose.set(sample_pose)
    print robot.knowledge.pose.keys()


    # delete
    print "\nDelete that new position: " 
    robot.knowledge.pose.delete('somewhere-near-the-kitchen')
    print robot.knowledge.pose.keys()


    # map name
    print "\nWhich map file are you using right now?: " 
    print robot.knowledge.pose.map_name()

    # save file
    print "\nDelete all data, add something and save a new file: " 
    for key in robot.knowledge.pose.keys():
        robot.knowledge.pose.delete(key)
    robot.knowledge.pose.set(sample_pose)
    robot.knowledge.pose.save_to_map('empty_map.sem_map')
    print robot.knowledge.pose.keys()
    print robot.knowledge.pose.map_name()

    # load map (current one)
    print "\nRestore previous data: " 
    robot.knowledge.pose.load_from_map()
    print robot.knowledge.pose.keys()

    # load map (another one)
    map_name = robot.knowledge.pose.map_name()
    print "\nLoad dummy map: " 
    robot.knowledge.pose.load_from_map('empty_map.sem_map')
    print robot.knowledge.pose.keys()

    # load map (restore again)
    print "\nRestore previous data again: " 
    robot.knowledge.pose.load_from_map(map_name)
    print robot.knowledge.pose.keys()

    print "--------------------------------------------------------------------"

  
    robot.knowledge.overview()

    