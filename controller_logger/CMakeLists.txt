cmake_minimum_required(VERSION 3.10)
project(controller_logger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(forward_command_controller REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  controller_interface
  hardware_interface
  forward_command_controller
  pluginlib
  realtime_tools
  std_msgs
)

# Generate parameter library
generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/controller_logger_parameters.yaml
)

# Add the main library
add_library(${PROJECT_NAME} SHARED
  src/controller_logger.cpp
)

# Link the parameter library manually
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_parameters
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export the plugin description file
pluginlib_export_plugin_description_file(controller_interface controller_logger_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()