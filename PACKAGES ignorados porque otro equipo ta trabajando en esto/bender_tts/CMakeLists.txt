cmake_minimum_required(VERSION 2.8.3)
project(bender_tts)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  rospy
  sound_play
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
  uchile_util
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    diagnostic_msgs
    roscpp
    rospy
    sound_play
    std_msgs
    std_srvs
    uchile_msgs
    uchile_srvs
)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# - - - tts - - -
add_executable(tts src/synthesizer.cpp)
add_dependencies(tts ${catkin_EXPORTED_TARGETS})
target_link_libraries(tts ${catkin_LIBRARIES})

