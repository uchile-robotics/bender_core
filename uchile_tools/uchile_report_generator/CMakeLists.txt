cmake_minimum_required(VERSION 2.8.3)
project(uchile_report_generator)

find_package(catkin REQUIRED COMPONENTS
  uchile_msgs
  uchile_srvs
  cv_bridge
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  uchile_util
)

find_package(OpenCV  REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uchile_report_generator
  CATKIN_DEPENDS uchile_msgs uchile_srvs cv_bridge nav_msgs roscpp roslib sensor_msgs std_msgs std_srvs uchile_util
  DEPENDS opencv2
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${uchile_util_INCLUDE_DIRS}
)

add_executable(report_generator src/report_generator.cpp)
add_dependencies(report_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(report_generator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
