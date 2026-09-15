cmake_minimum_required(VERSION 3.8)
project(bender_hardware_interfaces)

# Forzar C++17 o superior (Requerido por ROS 2 Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# AriaCoda se instala vía "make install" (ver Dockerfile) directamente en
# /usr/local/{include,lib}, sin config de CMake ni de pkg-config propia.
# Igual que en rosaria2, hay que agregar el rpath a mano para que el .so
# resultante encuentre libAria en tiempo de ejecución (dlopen vía pluginlib),
# aunque /usr/local/lib sea una ruta "del sistema".
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath -Wl,/usr/local/lib")

# 1. Buscar dependencias del sistema
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(aria REQUIRED)  --> system package, not configured?

# 2. Declarar la librería compartida (El Plugin)
# Ajusta "src/encoder_interface.cpp" si tu archivo está en otra ruta
add_library(${PROJECT_NAME} SHARED
  src/encoder_interface.cpp
  src/pioneer_interface.cpp
)

# 3. Vincular dependencias a la librería
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:/usr/local/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  Aria
)
# target_link_libraries(${PROJECT_NAME} /usr/local/lib/libAria.so)

# 4. EXPORTAR EL XML DEL PLUGIN (Crucial para pluginlib)
pluginlib_export_plugin_description_file(hardware_interface bender_hardware_interfaces.xml)

# 5. Instalar los binarios y cabeceras
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# 6. Exportar dependencias aguas abajo
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
