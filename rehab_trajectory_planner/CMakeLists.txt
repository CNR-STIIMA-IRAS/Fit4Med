cmake_minimum_required(VERSION 3.5)
project(rehab_trajectory_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
)

link_directories(/usr/local/lib)

# Create executable
add_executable(rehab_trajectory_planner_node src/rehab_trajectory_planner.cpp)

# Specify dependencies for the target
ament_target_dependencies(
  rehab_trajectory_planner_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  yaml-cpp
)

# Link YAML-CPP library explicitly
target_link_libraries(rehab_trajectory_planner_node yaml-cpp)
target_link_libraries(rehab_trajectory_planner_node sisl)

# Install targets
install(TARGETS
  rehab_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# Mark package as ament package
ament_package()
