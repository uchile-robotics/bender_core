cmake_minimum_required(VERSION 2.8.3)
project(bender_maps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  bender_msgs
  bender_srvs
  bender_utils
  geometry_msgs
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.7 REQUIRED COMPONENTS common)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_nav
  CATKIN_DEPENDS roscpp roslib rospy std_msgs std_srvs bender_msgs bender_srvs bender_utils geometry_msgs sensor_msgs tf pcl_ros
  DEPENDS pcl
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bender_utils_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Map Analyzer
add_executable(map_analyzer src/MapAnalyzer.cpp)
add_dependencies(map_analyzer ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_analyzer ${catkin_LIBRARIES})
