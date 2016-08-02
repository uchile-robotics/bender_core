cmake_minimum_required(VERSION 2.8.3)
project(bender_head)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  roscpp
  rospy
  std_msgs
  std_srvs
)

add_message_files(FILES
  ExpressionCommand.msg
  )

generate_messages(DEPENDENCIES bender_msgs)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS bender_msgs bender_srvs roscpp rospy std_msgs std_srvs
)


