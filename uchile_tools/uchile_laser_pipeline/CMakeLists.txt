cmake_minimum_required(VERSION 2.8.3)
project(uchile_laser_pipeline)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters
  laser_geometry pluginlib angles)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system signals)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uchile_laser_pipeline
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
)

##############################################################################
# Build
##############################################################################

add_library(uchile_laser_pipeline src/laser_scan_filters.cpp src/self_filter.cpp)
target_link_libraries(uchile_laser_pipeline ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(laser_scan_limiter src/laser_scan_limiter.cpp)
target_link_libraries(laser_scan_limiter ${catkin_LIBRARIES})