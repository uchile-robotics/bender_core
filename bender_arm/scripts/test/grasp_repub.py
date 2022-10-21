#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

def callback(data):
    rospy.loginfo("Marker Received, republishing...")
    while not rospy.is_shutdown():
        pub.publish(data)
        rate.sleep()    
def echo():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    


    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback)

    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node('marker_repub', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    pub = rospy.Publisher('/detect_grasps/plot_grasps', MarkerArray, queue_size=10)
    
    rospy.loginfo("Node Ready")
    echo()
