#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PolygonStamped


class FootprintGeneratorNode(object):
    

    def __init__(self):
        
        # ros interface
        self.pub = rospy.Publisher('footprint', PolygonStamped, queue_size=10)
        self.tf_listener = tf.TransformListener()

        # if (!psw.getParameter("footprint", _footprint, _footprint)) {
#         ROS_ERROR(
#             "The 'footprint' parameter is not set. It corresponds to a polygon defined as a list of lists."
#             " e.g.: [[x1, y1], [x2, y2], ...]");
#         failes = true;
#     }
#
#     if (!psw.getParameter("footprint_frame", _footprint_frame, "")) {
#     ROS_ERROR("The 'footprint_frame' parameter is not set. It is the footprint frame used to transform"
#               " the 'footprint' polygon into the laser scan frame. e.g.: base_link");
#     failes = true;
# }

        # params
        self.footprint = rospy.get_param("~footprint")
        self.footprint_frame = rospy.get_param("~footprint_frame")
        self.target_frames = rospy.get_param("~target_frames")
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.1)

        # setup
        rospy.loginfo("Waiting for initial transform from frames '" + str(self.target_frames) + "' to '" + self.footprint_frame + "'")
        while not rospy.is_shutdown():
            try:
                for target_frame in self.target_frames:
                    self.tf_listener.waitForTransform(target_frame, self.footprint_frame, rospy.Time(), rospy.Duration(2))
                break
            except tf.Exception:
                rospy.logwarn("Unavailable transform from frame '" + str(self.target_frame) + "' to '" + self.footprint_frame + "'. Retrying ...")

        rospy.loginfo("Ready to work ...")


    def spin(self):
        
        try:
            for target_frame in self.target_frames:
                
                # lookup
                now = rospy.Time.now()
                self.tf_listener.waitForTransform(target_frame, self.footprint_frame, now, rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform(target_frame, self.footprint_frame, rospy.Time(0))

                
                # generate footprint
                # print trans
                # print rot

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        except tf.Exception:
            rospy.logwarn("TF Exception. Possible clock reset.")


def main():
    rospy.init_node('footprint_generator')
    node = FootprintGeneratorNode()
    
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass