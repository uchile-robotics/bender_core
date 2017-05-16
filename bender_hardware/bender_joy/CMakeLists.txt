cmake_minimum_required(VERSION 2.8.3)
project(bender_joy)

find_package(catkin REQUIRED COMPONENTS
  uchile_msgs
  uchile_srvs
  joy
  roscpp
  sensor_msgs
  std_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES bender_joy
  CATKIN_DEPENDS uchile_msgs uchile_srvs joy roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
)
