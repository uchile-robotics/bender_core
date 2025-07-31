cmake_minimum_required(VERSION 3.8)
project(bender_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(librealsense2_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
) 

install(PROGRAMS
  scripts/laser_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
