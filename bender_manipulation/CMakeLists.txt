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
    scripts/saludo_node.py
    scripts/camera_time_bridge_node.py
    scripts/grasp_pick_node.py
    scripts/graspgen_pick_bridge_node.py
    scripts/sam3_segment_node.py
    scripts/pub_pose.py
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
