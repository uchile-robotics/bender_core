#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv

from geometry_msgs.msg import PoseStamped
from manipulator import Manipulator
from moveit_commander import (
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
)

if __name__ == "__main__":
    rospy.init_node("workspace_analysis", anonymous=True)
    roscpp_initialize(sys.argv)
    
    pose_pub = rospy.Publisher('grasp_pose', PoseStamped, queue_size=10)

    arm = Manipulator()
    
    step = 0.05
    X = np.arange(0.0-step, 0.9+step, step)
    Y = np.arange(-0.7-step, 1.1+step, step)
    Z = np.arange(0.2-step, 2.0+step, step)

    x_res = []
    y_res = []
    z_res = []

    rospy.loginfo("Starting Analysis...")
    
    i=0
    for x in X:
        for y in Y:
            for z in Z:
                i+=1
                rospy.loginfo("it. n {} ({},{},{})".format(i,x,y,z))
                spos=PoseStamped()
                spos.header.frame_id="bender/base_link"

                #Position 
                spos.pose.position.x, spos.pose.position.y, spos.pose.position.z = x, y, z
                #Orientation
                spos.pose.orientation.x, spos.pose.orientation.y, spos.pose.orientation.z, spos.pose.orientation.w = 0.0, 0.0, 0.0, 1.0

                pose_pub.publish(spos)

                if arm.try_pose(spos):
                    x_res.append(x)
                    y_res.append(y)
                    z_res.append(z)

    rospy.loginfo("Analysis Finished")

    file = open("/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/bender_arm/scripts/workspace.csv", "w")
    writer = csv.writer(file)
    writer.writerow(["x","y","z"])
    writer.writerows(zip(x_res, y_res, z_res))
    file.close()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d') 

    ax.scatter3D(x_res, y_res, z_res)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

    rospy.spin()