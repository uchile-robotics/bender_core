cmake_minimum_required(VERSION 2.8.3)
project(bender_ssl)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roslib
  rospy
  std_msgs
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

#Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
  FILES
  Locate.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
  actionlib_msgs
  visualization_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES barbie_rviz
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    rospy
    std_msgs
    visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
