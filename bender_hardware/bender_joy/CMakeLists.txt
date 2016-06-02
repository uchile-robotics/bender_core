cmake_minimum_required(VERSION 2.8.3)
project(bender_joy)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  joy
  roscpp
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES bender_joy
  CATKIN_DEPENDS bender_msgs bender_srvs joy roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
)

# Joystick Complete
add_executable(joystick src/joystick.cpp)
add_dependencies(joystick ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick ${catkin_LIBRARIES})
