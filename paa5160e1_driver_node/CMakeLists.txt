# Copyright (c) 2025 NITK.K ROS-Team.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(paa5160e1_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

set(TARGET paa5160e1_driver_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED
  src/${TARGET}.cpp
  src/odom_class.cpp
)

rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::Paa5160e1DriverNode"
  EXECUTABLE ${TARGET}_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
