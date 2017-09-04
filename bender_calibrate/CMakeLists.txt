cmake_minimum_required(VERSION 2.8.3)
project(bender_calibrate)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
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
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    roslib
    sensor_msgs
    std_msgs
    uchile_msgs
    uchile_srvs
  DEPENDS opencv2
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)
