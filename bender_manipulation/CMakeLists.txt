cmake_minimum_required(VERSION 3.8)
project(bender_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias básicas
find_package(ament_cmake REQUIRED)

# --- NUEVO: Dependencias obligatorias para C++ ---
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# -------------------------------------------------

# --- Instalación de recursos ---

# Launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Config files (incluye params.yaml)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Scripts Python ejecutables
install(
  PROGRAMS
    scripts/dynamixel_node.py
    scripts/dynamixel_vel_node.py
    scripts/oddrive.py
    scripts/dynamixel_command.py
    scripts/shoulder_control.py
    scripts/dynamixel_trajectory_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Compilación y enlace del nodo C++
add_executable(serial_encoder_node src/serial_encoder_node.cpp)
ament_target_dependencies(serial_encoder_node rclcpp std_msgs)

install(TARGETS
  serial_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Tests/linters opcionales
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
