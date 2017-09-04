cmake_minimum_required(VERSION 2.8.3)
project(bender_fieldbus)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS
		dynamixel_controllers
		roscpp
		rospy
		std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
