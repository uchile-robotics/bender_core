cmake_minimum_required(VERSION 2.8.3)
project(bender_sensors)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  uchile_msgs
  uchile_srvs
  uchile_util
)

find_package(OpenCV  REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES bender_sensors
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    uchile_msgs
    uchile_srvs
    uchile_util
  DEPENDS opencv2
)

###########
## Build ##
###########

include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${uchile_util_INCLUDE_DIRS}
)

## Declare a cpp executable

# laser_inverter
add_executable(laser_inverter src/laser_inverter.cpp)
add_dependencies(laser_inverter ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_inverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
