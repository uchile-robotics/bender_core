cmake_minimum_required(VERSION 2.8.3)
project(uchile_safety)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  geometry_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()