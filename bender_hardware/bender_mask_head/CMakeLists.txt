cmake_minimum_required(VERSION 2.8.3)
project(bender_mask_head)


find_package(catkin REQUIRED COMPONENTS
  bender_fieldbus
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
)


catkin_python_setup()


