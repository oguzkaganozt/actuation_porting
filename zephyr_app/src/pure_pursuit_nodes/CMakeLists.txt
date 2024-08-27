# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit_nodes/CMakeLists.txt
# In open-source project: Autoware.Auto
# Original file: Copyright (c) 2019, the Autoware Foundation
# Modifications: Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit_nodes)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  include/pure_pursuit_nodes/pure_pursuit_node.hpp
  include/pure_pursuit_nodes/visibility_control.hpp
  src/pure_pursuit_node.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${motion_common_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CycloneDDS_INCLUDE_DIR}
)

if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  set(PURE_PURSUIT_NODE_TEST pure_pursuit_integration_test_exe)
  ament_auto_add_executable(${PURE_PURSUIT_NODE_TEST} test/src/pure_pursuit_integration_test.cpp)
endif()

## install stuff
ament_auto_package(
)
