# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(isaac_ros_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(msgpack REQUIRED)
ament_auto_find_build_dependencies()

# Core logic lives in a library so both the node and the unit tests link it
# (matches the Bazel pose_reset_node_lib seam).
ament_auto_add_library(pose_reset_node_core SHARED src/pose_reset_node.cpp)
target_include_directories(pose_reset_node_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# msgpack is a plain CMake/rosdep dependency (not ament), so link it explicitly.
target_link_libraries(pose_reset_node_core Eigen3::Eigen msgpackc)

ament_auto_add_executable(pose_reset_node src/pose_reset_main.cpp)
target_link_libraries(pose_reset_node pose_reset_node_core)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_copyright test/test_copyright.py TIMEOUT 60)
  ament_add_pytest_test(test_flake8 test/test_flake8.py TIMEOUT 60)
  ament_add_pytest_test(test_pep257 test/test_pep257.py TIMEOUT 60)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_reset_node test/test_pose_reset_node.cpp)
  target_link_libraries(test_pose_reset_node pose_reset_node_core)
endif()

# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch)
