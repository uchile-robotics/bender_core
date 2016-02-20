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
	include
	${catkin_INCLUDE_DIRS})

# Joystick Nav
add_executable(joystick_nav src/joystick_nav.cpp)
add_dependencies(joystick_nav ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_nav ${catkin_LIBRARIES})

# Joystick Complete
add_executable(joystick_complete src/joy_complete.cpp)
add_dependencies(joystick_complete ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_complete ${catkin_LIBRARIES})


# Joystick Inauguracion Beucheff
add_executable(joystick_inauguracion src/pres_arms_joy.cpp)
add_dependencies(joystick_inauguracion ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_inauguracion ${catkin_LIBRARIES})


# Face Joystick
add_executable(joystick_face src/joystick_face.cpp)
add_dependencies(joystick_face ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_face ${catkin_LIBRARIES})

# Face Auto Sequence Joystick
add_executable(joy_autoface src/joy_autoface.cpp)
add_dependencies(joy_autoface ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_autoface ${catkin_LIBRARIES})

# Base Arm
add_executable(joy_baseArm src/joy_baseArm.cpp)
add_dependencies(joy_baseArm ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_baseArm ${catkin_LIBRARIES})

# Upper Arms Dance
add_executable(joy_arms_dance src/joy_arms_dance.cpp)
add_dependencies(joy_arms_dance ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_arms_dance ${catkin_LIBRARIES})

# Upper Arms Complete
add_executable(joy_arms_complete src/simple_arms_joy.cpp)
add_dependencies(joy_arms_complete ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_arms_complete ${catkin_LIBRARIES})

