cmake_minimum_required(VERSION 2.8.3)
project(bender_msgs)

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

add_message_files(
  FILES
  Emotion.msg
  Expression.msg
  Mouth.msg
  Joyino.msg
  Face.msg
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    std_srvs
    uchile_msgs
    uchile_srvs
    message_runtime
)

###################################
## catkin specific configuration ##
###################################




