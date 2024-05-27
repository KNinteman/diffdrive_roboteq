cmake_minimum_required(VERSION 3.8)
project(diffdrive_roboteq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(roboteq_interfaces REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add compile directive for shared library
add_library(diffdrive_roboteq 
  SHARED
  src/diffdrive_roboteq.cpp
  src/query_config.cpp
)

install(DIRECTORY
  src config
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(
  diffdrive_roboteq
  PRIVATE
  include
)

target_link_libraries(diffdrive_roboteq yaml-cpp)

# Add ament dependencies
ament_target_dependencies(diffdrive_roboteq
  hardware_interface
  roboteq_interfaces
  pluginlib
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
)

install(
  DIRECTORY include/
  DESTINATION include/diffdrive_roboteq
)

install(TARGETS diffdrive_roboteq
  EXPORT diffdrive_roboteq
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export pluginlib description file
pluginlib_export_plugin_description_file(hardware_interface diffdrive_roboteq.xml)


ament_export_libraries(
  diffdrive_roboteq
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()