cmake_minimum_required(VERSION 2.8.3)
project(bender_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
