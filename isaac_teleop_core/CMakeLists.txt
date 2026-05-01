# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(isaac_teleop_core)

find_package(ament_cmake REQUIRED)

set(ISAACTELEOP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/IsaacTeleop")
set(TELEOP_ROS2_PUBLISHER_SCRIPT
    "${ISAACTELEOP_SRC}/examples/teleop_ros2/python/teleop_ros2_publisher.py")

if(NOT EXISTS "${TELEOP_ROS2_PUBLISHER_SCRIPT}")
  message(FATAL_ERROR
    "Missing Teleop ROS2 publisher script: ${TELEOP_ROS2_PUBLISHER_SCRIPT}\n"
    "Ensure the IsaacTeleop submodule is initialized.")
endif()

# Install only the ROS2 publisher entrypoint. The `isaacteleop` Python package
# is provided by the pip shim dependency at runtime.
install(PROGRAMS
  "${TELEOP_ROS2_PUBLISHER_SCRIPT}"
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_ros2_publisher
)

ament_package()
