cmake_minimum_required(VERSION 2.8.3)
project(bender_sound)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  roscpp
  rospy
  roslib
  std_msgs
  std_srvs
  sound_play
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS bender_msgs bender_srvs roscpp rospy roslib std_msgs std_srvs sound_play
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(player src/sound_player.cpp)
add_dependencies(player ${catkin_EXPORTED_TARGETS})
target_link_libraries(player ${catkin_LIBRARIES})
