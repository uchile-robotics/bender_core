cmake_minimum_required(VERSION 2.8.3)
project(bender_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  angles
  bender_description
  cmake_modules
  control_msgs
  control_toolbox
  controller_interface
  controller_manager
  dynamic_reconfigure
  effort_controllers
  forward_command_controller
  gazebo_ros
  hardware_interface
  message_generation
  realtime_tools
  roscpp
  trajectory_msgs
  urdf
  xacro
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    control_msgs
    control_toolbox
    controller_interface
    controller_interface
    forward_command_controller
    hardware_interface
    realtime_tools
    realtime_tools
    trajectory_msgs
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bender_gripper_action_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

