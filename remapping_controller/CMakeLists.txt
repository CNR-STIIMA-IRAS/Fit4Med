cmake_minimum_required(VERSION 3.16)
project(remapping_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  remapping_controller_parameters
  src/remapping_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/remapping_controller.cpp
)

target_compile_features(remapping_controller PUBLIC cxx_std_17)
target_include_directories(remapping_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/remapping_controller>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  remapping_controller_parameters
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  hardware_interface::hardware_interface
  ${trajectory_msgs_TARGETS}
)


pluginlib_export_plugin_description_file(controller_interface remapping_controller.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/remapping_controller
)

install(TARGETS remapping_controller remapping_controller_parameters
  EXPORT export_remapping_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_remapping_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
