cmake_minimum_required(VERSION 2.8.3)
project(bender_tts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
  diagnostic_msgs
  sound_play
  bender_utils
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS uchile_msgs uchile_srvs diagnostic_msgs roscpp rospy sound_play std_msgs std_srvs
)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${bender_utils_INCLUDE_DIRS}
)

# - - - tts - - -
add_executable(tts src/synthesizer.cpp)
add_dependencies(tts ${catkin_EXPORTED_TARGETS})
target_link_libraries(tts ${catkin_LIBRARIES})

