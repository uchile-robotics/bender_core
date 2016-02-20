cmake_minimum_required(VERSION 2.8.3)
project(bender_sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  bender_msgs
  bender_srvs
  cv_bridge
  image_transport
  bender_utils
)

find_package(OpenCV  REQUIRED)

find_path(LIBUSB_INCLUDEDIR
    NAMES libusb.h
    HINTS /usr/include/libusb-1.0
)
find_library(LIBUSB_LIBRARIES
    NAMES usb-1.0
    HINTS /usr/lib/ /usr/x86_64-linux-gnu/
    PATH_SUFFIXES lib
)

message(STATUS ${LIBUSB_INCLUDEDIR})
message(STATUS ${LIBUSB_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_sensors
  CATKIN_DEPENDS bender_msgs bender_srvs cv_bridge image_transport roscpp sensor_msgs std_msgs bender_utils
  DEPENDS opencv2 LIBUSB_LIBRARIES
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}
    ${bender_utils_INCLUDE_DIRS}
)

## Declare a cpp executable
# laser_merger
#add_executable(laser_merger src/laser_merger.cpp)
#add_dependencies(laser_merger ${catkin_EXPORTED_TARGETS})
#target_link_libraries(laser_merger ${catkin_LIBRARIES})

# laser_inverter
add_executable(laser_inverter src/laser_inverter.cpp)
add_dependencies(laser_inverter ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_inverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# camera_service
add_executable(camera_service src/camera_service_server.cpp)
add_dependencies(camera_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_service ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(camera_service_client src/camera_service_client.cpp)
add_dependencies(camera_service_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_service_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## kinect_aux
add_executable(kinect_aux_node src/kinect/kinect_aux.cpp)
add_dependencies(kinect_aux_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect_aux_node
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    ${LIBUSB_LIBRARIES} 
)
