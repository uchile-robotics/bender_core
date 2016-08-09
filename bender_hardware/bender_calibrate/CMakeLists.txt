cmake_minimum_required(VERSION 2.8.3)
project(bender_calibrate)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  cv_bridge
  camera_calibration
  geometry_msgs
  image_transport
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(OpenCV  REQUIRED)

catkin_python_setup()

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fpermissive")

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES bender_fun
  CATKIN_DEPENDS bender_msgs bender_srvs roscpp roslib std_msgs
  DEPENDS opencv2
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)