cmake_minimum_required(VERSION 3.8)
project(bender_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Executable
add_executable(teleop_node src/joystick_node.cpp)
ament_target_dependencies(teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
