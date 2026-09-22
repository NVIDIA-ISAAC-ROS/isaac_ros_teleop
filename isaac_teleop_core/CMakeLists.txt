# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(isaac_teleop_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(teleop_ros2_interfaces REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

set(ISAACTELEOP_ROS_PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
set(ISAACTELEOP_ROS_PYTHON_VERSION
    "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
set(ISAACTELEOP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/IsaacTeleop")
set(ISAACTELEOP_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/IsaacTeleop")
set(ISAACTELEOP_UV_CACHE_DIR "${ISAACTELEOP_BUILD_DIR}/uv_cache")
set(ISAACTELEOP_UV_PYTHON_INSTALL_DIR "${ISAACTELEOP_BUILD_DIR}/uv_python")
set(ISAACTELEOP_WHEEL_DIR "${ISAACTELEOP_BUILD_DIR}/wheels")
set(ISAACTELEOP_WHEEL_STAMP "${ISAACTELEOP_BUILD_DIR}/python_wheel.stamp")
set(ISAACTELEOP_UV_ENV
    "GIT_ROOT=${ISAACTELEOP_SRC}"
    "UV_CACHE_DIR=${ISAACTELEOP_UV_CACHE_DIR}"
    "UV_PYTHON_INSTALL_DIR=${ISAACTELEOP_UV_PYTHON_INSTALL_DIR}"
    "UV_PYTHON_INSTALL_BIN=0")
set(ISAACTELEOP_CMAKE_ARGS
    "-DCMAKE_INSTALL_PREFIX=${ISAACTELEOP_BUILD_DIR}/install"
    -DCMAKE_BUILD_TYPE=Release
    "-DISAAC_TELEOP_PYTHON_VERSION=${ISAACTELEOP_ROS_PYTHON_VERSION}"
    -DBUILD_PYTHON_BINDINGS=ON
    -DBUILD_EXAMPLES=OFF
    -DBUILD_EXAMPLE_TELEOP_ROS2=OFF
    -DBUILD_PLUGINS=OFF
    -DBUILD_PLUGIN_OAK_CAMERA=OFF
    -DBUILD_TESTING=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_CONFORMANCE_TESTS=OFF
    -DBUILD_API_LAYERS=OFF
    -DBUILD_VIZ=OFF
    -DENABLE_CLANG_FORMAT_CHECK=OFF
    -DENABLE_CLOUDXR_BUNDLE_CHECK=ON
    -DBUNDLE_ROBOTIC_GROUNDING=OFF)
set(TELEOP_ROS2_EXAMPLE_DIR "${ISAACTELEOP_SRC}/examples/teleop_ros2")
set(TELEOP_ROS2_PYTHON_DIR "${ISAACTELEOP_SRC}/examples/teleop_ros2/python")
set(TELEOP_ROS2_NODE_SCRIPT
    "${TELEOP_ROS2_PYTHON_DIR}/teleop_ros2_node.py")
set(TELEOP_ROS2_HELPER_MODULES
    "${TELEOP_ROS2_PYTHON_DIR}/assets.py"
    "${TELEOP_ROS2_PYTHON_DIR}/constants.py"
    "${TELEOP_ROS2_PYTHON_DIR}/geometry.py"
    "${TELEOP_ROS2_PYTHON_DIR}/messages.py"
    "${TELEOP_ROS2_PYTHON_DIR}/node_parameters.py"
    "${TELEOP_ROS2_PYTHON_DIR}/session_config.py"
    "${TELEOP_ROS2_PYTHON_DIR}/tensor_group_helpers.py"
    "${TELEOP_ROS2_PYTHON_DIR}/teleop_profiles.py")
set(TELEOP_ROS2_RETARGETERS_PACKAGE teleop_ros2_retargeters)
set(TELEOP_ROS2_RETARGETERS_DIR
    "${TELEOP_ROS2_PYTHON_DIR}/${TELEOP_ROS2_RETARGETERS_PACKAGE}")

foreach(required_path IN ITEMS
    "${ISAACTELEOP_SRC}/CMakeLists.txt"
    "${TELEOP_ROS2_NODE_SCRIPT}"
    ${TELEOP_ROS2_HELPER_MODULES}
    "${TELEOP_ROS2_EXAMPLE_DIR}/assets"
    "${TELEOP_ROS2_EXAMPLE_DIR}/configs"
    "${TELEOP_ROS2_RETARGETERS_DIR}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR
      "Missing Teleop ROS 2 Python source: ${required_path}\n"
      "Ensure the IsaacTeleop submodule is initialized.")
  endif()
endforeach()

file(GLOB_RECURSE ISAACTELEOP_SOURCE_DEPENDENCIES
  LIST_DIRECTORIES false
  CONFIGURE_DEPENDS
  "${ISAACTELEOP_SRC}/CMakeLists.txt"
  "${ISAACTELEOP_SRC}/VERSION"
  "${ISAACTELEOP_SRC}/cmake/*"
  "${ISAACTELEOP_SRC}/deps/*"
  "${ISAACTELEOP_SRC}/src/*"
)

add_custom_command(
  OUTPUT "${ISAACTELEOP_WHEEL_STAMP}"
  COMMAND ${CMAKE_COMMAND} -E rm -rf "${ISAACTELEOP_WHEEL_DIR}"
  COMMAND ${CMAKE_COMMAND}
    -E env
    ${ISAACTELEOP_UV_ENV}
    ${CMAKE_COMMAND}
    -S "${ISAACTELEOP_SRC}"
    -B "${ISAACTELEOP_BUILD_DIR}"
    ${ISAACTELEOP_CMAKE_ARGS}
  COMMAND ${CMAKE_COMMAND}
    -E env
    ${ISAACTELEOP_UV_ENV}
    ${CMAKE_COMMAND}
    --build "${ISAACTELEOP_BUILD_DIR}"
    --target python_wheel
    --parallel
  COMMAND ${CMAKE_COMMAND} -E touch "${ISAACTELEOP_WHEEL_STAMP}"
  DEPENDS
    ${ISAACTELEOP_SOURCE_DEPENDENCIES}
  COMMENT "Building isaacteleop Python wheel from source"
  VERBATIM
)

add_custom_target(isaacteleop_python_wheel ALL
  DEPENDS "${ISAACTELEOP_WHEEL_STAMP}"
)

ament_python_install_package(
  ${TELEOP_ROS2_RETARGETERS_PACKAGE}
  PACKAGE_DIR "${TELEOP_ROS2_RETARGETERS_DIR}"
)

install(CODE "
  set(_isaacteleop_install_prefix \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\")
  file(MAKE_DIRECTORY \"\${_isaacteleop_install_prefix}/${PYTHON_INSTALL_DIR}\")
  execute_process(
    COMMAND \"${ISAACTELEOP_ROS_PYTHON_EXECUTABLE}\" -m pip install
      --disable-pip-version-check
      --target \"\${_isaacteleop_install_prefix}/${PYTHON_INSTALL_DIR}\"
      --upgrade
      --ignore-installed
      --no-index
      --find-links \"${ISAACTELEOP_WHEEL_DIR}\"
      --no-deps
      isaacteleop[cloudxr,retargeters]
    RESULT_VARIABLE _isaacteleop_install_result
    OUTPUT_VARIABLE _isaacteleop_install_stdout
    ERROR_VARIABLE _isaacteleop_install_stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _isaacteleop_install_result EQUAL 0)
    message(FATAL_ERROR
      \"Failed to install source-built isaacteleop wheel from ${ISAACTELEOP_WHEEL_DIR}\\n\"
      \"stdout: \${_isaacteleop_install_stdout}\\n\"
      \"stderr: \${_isaacteleop_install_stderr}\")
  endif()
")

# Install the ROS 2 entrypoint and its local helpers.
install(FILES
  ${TELEOP_ROS2_HELPER_MODULES}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  "${TELEOP_ROS2_NODE_SCRIPT}"
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_ros2_node
)

# Match upstream teleop_ros2 packaging: node_parameters.py defaults its
# config_asset_root to the example root (parent of python/), so install these
# under lib/ beside isaac_teleop_core/.
install(DIRECTORY
  "${TELEOP_ROS2_EXAMPLE_DIR}/assets"
  "${TELEOP_ROS2_EXAMPLE_DIR}/configs"
  DESTINATION lib
)

ament_package()
