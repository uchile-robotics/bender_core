cmake_minimum_required(VERSION 2.8.3)
project(bender_stage)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  uchile_msgs
  uchile_srvs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		std_msgs
)

