#!/usr/bin/env python

import roslib
roslib.load_manifest('uchile_util')
import rospy
import sys

def ServiceProxy(name, service_class):
    
    client = rospy.ServiceProxy(name, service_class)
    #rospy.loginfo("Waiting for service: '" + str(name) + "' to come up")
    
    while True:
        try:
            client.wait_for_service(timeout=2.0)
            break
        except rospy.ROSInterruptException:
            sys.exit()
        except rospy.ROSException:
            rospy.logwarn("Waiting for service: '" + str(name) + "' to come up")
        try:
            rospy.sleep(0.5)
        except rospy.ROSInterruptException:
            sys.exit()
    
    rospy.loginfo("service '" + str(client.resolved_name) + "' is READY")
    return client

    
    