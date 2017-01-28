#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Polygon, PolygonStamped, PointStamped, Point32
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError 
import numpy as np


class FootprintGeneratorNode(object):

    def __init__(self):
        
        # ros interface
        self.pub = rospy.Publisher('footprint', Polygon, queue_size=10)
        self.pub_stamped = rospy.Publisher('footprint_stamped', PolygonStamped, queue_size=10)
        self.pub_original_stamped = rospy.Publisher('footprint_base', PolygonStamped, queue_size=10)
        self.tf_listener = tf.TransformListener()

        # params
        self.footprint = rospy.get_param("~footprint")
        self.footprint_frame = rospy.get_param("~footprint_frame")
        self.target_frames = rospy.get_param("~target_frames")
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.1)

        # build footprint (for assertion purposes and base footprint generation)
        base_hull = self.build_base_hull(self.footprint)
        self.base_msg = PolygonStamped()
        self.base_msg.header.frame_id = self.footprint_frame
        self.base_msg.polygon = self.build_polygon_msg(base_hull)

        # setup
        rospy.loginfo("Waiting for initial transform from frames '" +
                      str(self.target_frames) + "' to '" + self.footprint_frame + "'")
        while not rospy.is_shutdown():
            success = True
            for target_frame in self.target_frames:
                try:
                    self.tf_listener.waitForTransform(
                        target_frame, self.footprint_frame, rospy.Time(), rospy.Duration(2))
                except tf.Exception:
                    rospy.logwarn("Unavailable transform from frame '" +
                                  str(target_frame) + "' to '" + self.footprint_frame + "'. Retrying ...")
                    success = False
            if success:
                break
        
        rospy.loginfo("Ready to work ...")

    def spin(self):

        # base convex hull
        hull = self.build_base_hull(self.footprint)

        # add points to the hull
        now = rospy.Time.now()
        try:
            for target_frame in self.target_frames:
                joint = PointStamped()
                joint.header.frame_id = target_frame
                joint.header.stamp = now

                # transform
                self.tf_listener.waitForTransform(target_frame, self.footprint_frame, now, rospy.Duration(1))
                tf_joint = self.tf_listener.transformPoint(self.footprint_frame, joint)

                # append joint and its inflated version
                r = self.inflation_radius
                np_joint = np.array([
                    (tf_joint.point.x, tf_joint.point.y),
                    (tf_joint.point.x + r, tf_joint.point.y + r),
                    (tf_joint.point.x + r, tf_joint.point.y - r),
                    (tf_joint.point.x - r, tf_joint.point.y + r),
                    (tf_joint.point.x - r, tf_joint.point.y - r)
                ])
                hull.add_points(np_joint)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        except tf.Exception:
            rospy.logwarn("TF Exception. Possible clock reset.")

        hull.close()

        # generate and publish Polygon, PolygonStamped
        polygon_msg = self.build_polygon_msg(hull)
        polygon_stamped_msg = PolygonStamped()
        polygon_stamped_msg.header.frame_id = self.footprint_frame
        polygon_stamped_msg.header.stamp = now
        polygon_stamped_msg.polygon = polygon_msg
        self.pub.publish(polygon_msg)
        self.pub_stamped.publish(polygon_stamped_msg)

        # publish base footprint
        self.base_msg.header.stamp = now
        self.pub_original_stamped.publish(self.base_msg)

    @staticmethod
    def build_polygon_msg(hull):
        msg = Polygon()
        for vertex_idx in hull.vertices:
            msg.points.append(Point32(
                hull.points[vertex_idx, 0],
                hull.points[vertex_idx, 1],
                0
            ))
        return msg

    @staticmethod
    def build_base_hull(points):
        hull = np.zeros((len(points), 2))
        for idx in range(len(points)):
            hull[idx, 0] = points[idx][0]
            hull[idx, 1] = points[idx][1]

        try:
            hull = ConvexHull(points, incremental=True)
        except (QhullError, ValueError) as e:
            rospy.logerr("Failed to create the convex hull for the given robot footprint.")
            raise e
        return hull


def main():
    rospy.init_node('footprint_generator')
    node = FootprintGeneratorNode()
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
