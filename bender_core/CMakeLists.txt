cmake_minimum_required(VERSION 2.8.3)
project(bender_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  bender_msgs
  bender_srvs
  rospy
  std_msgs
  std_srvs
)

#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS bender_msgs bender_srvs rospy std_msgs std_srvs
)
