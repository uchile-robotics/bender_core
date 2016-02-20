cmake_minimum_required(VERSION 2.8.3)
project(bender_stage)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)


###################################
## catkin specific configuration ##
###################################
catkin_package()

