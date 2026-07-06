# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(isaac_teleop_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(ISAACTELEOP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/IsaacTeleop")
set(TELEOP_ROS2_PYTHON_DIR "${ISAACTELEOP_SRC}/examples/teleop_ros2/python")
set(TELEOP_ROS2_NODE_SCRIPT
    "${TELEOP_ROS2_PYTHON_DIR}/teleop_ros2_node.py")
set(TELEOP_ROS2_RETARGETERS_PACKAGE teleop_ros2_retargeters)
set(TELEOP_ROS2_RETARGETERS_DIR
    "${TELEOP_ROS2_PYTHON_DIR}/${TELEOP_ROS2_RETARGETERS_PACKAGE}")

foreach(required_path IN ITEMS
    "${TELEOP_ROS2_NODE_SCRIPT}"
    "${TELEOP_ROS2_RETARGETERS_DIR}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR
      "Missing Teleop ROS 2 Python source: ${required_path}\n"
      "Ensure the IsaacTeleop submodule is initialized.")
  endif()
endforeach()

ament_python_install_package(
  ${TELEOP_ROS2_RETARGETERS_PACKAGE}
  PACKAGE_DIR "${TELEOP_ROS2_RETARGETERS_DIR}"
)

# The `isaacteleop` Python package is provided by the pip shim dependency at
# runtime; this package installs the ROS 2 entrypoint and its local helpers.
install(PROGRAMS
  "${TELEOP_ROS2_NODE_SCRIPT}"
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_ros2_node
)

ament_package()
