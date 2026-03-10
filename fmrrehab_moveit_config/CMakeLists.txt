# Copyright 2026 CNR-STIIMA-IRAS
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fmrrehab_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
