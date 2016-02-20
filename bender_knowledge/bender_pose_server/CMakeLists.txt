cmake_minimum_required(VERSION 2.8.3)
project(bender_pose_server)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  bender_utils
  interactive_markers
)


# pkg-config support
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_pose_server
  CATKIN_DEPENDS bender_msgs bender_srvs geometry_msgs roscpp roslib rospy std_msgs std_srvs bender_utils interactive_markers
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${bender_utils_INCLUDE_DIRS}
)

# Pose Server
add_executable(pose_server src/PoseServer.cpp)
add_dependencies(pose_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_server ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

# rviz utility for pose management
add_executable(pose_server_marker_plugin src/PoseServerMarkerPlugin.cpp)
add_dependencies(pose_server_marker_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_server_marker_plugin ${catkin_LIBRARIES})
