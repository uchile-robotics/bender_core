cmake_minimum_required(VERSION 3.8)
project(bender_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(<dependency> REQUIRED)  # Agrega tus dependencias aquí

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instalar carpetas con archivos necesarios
install(DIRECTORY launch urdf config rviz worlds meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
