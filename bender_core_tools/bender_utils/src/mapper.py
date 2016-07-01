#!/usr/bin/env python
# by matias.pavez.b@gmail.com

import roslib
roslib.load_manifest('bender_utils')
import rospy
from bender_srvs.srv import String as string_srv

class Mapper():

    def __init__(self):
    
        self.server = rospy.Service('~get_class', string_srv, self.getClass)
        self.server = rospy.Service('~get_class_location', string_srv, self.getClassLocation)

        # mappings
        self.class_mapping = rospy.get_param('~classes')
        self.unknown_class = rospy.get_param('~unknown_class')

        rospy.loginfo("Mapper initialized OK")

    def getClass(self,req):

        for object_class in self.class_mapping:

            objects = rospy.get_param('~' + object_class)

            if req.data in objects:
                rospy.loginfo("class found: " + object_class + ", for item: " + req.data)
                return {'data':object_class}

        rospy.logwarn("class not found, for item: " + req.data)
        return {'data':self.unknown_class}

    def getClassLocation(self,req):

        # check unknown
        if self.unknown_class == req.data:
            if rospy.has_param('~' + self.unknown_class + '_location'):

                location = rospy.get_param('~' + self.unknown_class + '_location')
                rospy.loginfo("class location found: " + location + ", for class: " + req.data)
                return {'data':location}

        # check other classes
        for object_class in self.class_mapping:

            if object_class == req.data:
                if rospy.has_param('~' + object_class + '_location'):

                    location = rospy.get_param('~' + object_class + '_location')
                    rospy.loginfo("class location found: " + location + ", for class: " + req.data)
                    return {'data':location}

        rospy.logwarn("location not found, for class: " + req.data)
        return {'data':''}
        
def main():

    rospy.init_node('mapper')

    Mapper()
    rospy.spin()

if __name__ == '__main__':
    main()

