# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/controller_common_nodes/CMakeLists.txt
# In open-source project: Autoware.Auto
# Original file: Copyright (c) 2019, Christopher Ho
# Modifications: Copyright (c) 2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.6)

project(controller_common_nodes)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME}
  src/controller_common_nodes/controller_base_node.cpp)
# Setting a system include is needed to avoid having an explicit Eigen dependency.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${actuation_msgs_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ../../include
  ${CycloneDDS_INCLUDE_DIR}
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(controller_common_nodes_unit_tests
    test/bad_cases.cpp
    test/error_handling.cpp
    test/gtest_main.cpp
    test/sanity_check.cpp
    test/transform.cpp)
  target_compile_options(controller_common_nodes_unit_tests PRIVATE -Wno-double-promotion)
  target_link_libraries(controller_common_nodes_unit_tests ${PROJECT_NAME})
  add_dependencies(controller_common_nodes_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
