cmake_minimum_required(VERSION 2.8.3)
project(uchile_footprint_generator)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS rospy tf geometry_msgs)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
)

##############################################################################
# Build
##############################################################################
