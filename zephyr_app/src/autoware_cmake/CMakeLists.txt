# Based on: https://github.com/autowarefoundation/autoware_common/blob/ba4d3cc3f729ebedc16aba7d745bcc54fd935e61/autoware_cmake/CMakeLists.txt
    # In open-source project: autoware_common
    # Original file: Copyright (c) 2022, The Autoware Contributors
    # Modifications: Copyright (c) 2023, Arm Limited.
    # SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)

project(autoware_cmake NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "autoware_cmake-extras.cmake"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    cmake
)
