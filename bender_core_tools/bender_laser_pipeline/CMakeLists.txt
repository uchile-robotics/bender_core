cmake_minimum_required(VERSION 2.8.3)
project(bender_laser_pipeline)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs
  #DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})


#############
## Install ##
#############

