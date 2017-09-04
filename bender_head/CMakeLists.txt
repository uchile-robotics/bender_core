cmake_minimum_required(VERSION 2.8.3)
project(bender_head)

find_package(catkin REQUIRED COMPONENTS
  bender_fieldbus
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    std_srvs
    uchile_msgs
    uchile_srvs
)


