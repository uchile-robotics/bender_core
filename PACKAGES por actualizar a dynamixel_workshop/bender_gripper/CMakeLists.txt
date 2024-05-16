cmake_minimum_required(VERSION 2.8.3)
project(bender_gripper)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  bender_fieldbus
  cmake_modules
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/GripperParams.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    rospy
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
