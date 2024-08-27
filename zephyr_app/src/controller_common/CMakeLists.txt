# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/controller_common/CMakeLists.txt
# In open-source project: Autoware.Auto
# Original file: Copyright (c) 2019, Christopher Ho
# Modifications: Copyright (c) 2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.6)

project(controller_common)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# TODO(c.ho) this needs to be fixed
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# Build library
ament_auto_add_library(${PROJECT_NAME}
  src/controller_common/controller_base.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CycloneDDS_INCLUDE_DIR}
)

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit test
  apex_test_tools_add_gtest(controller_common_unit_tests
    test/gtest_main.cpp
    test/behavior.cpp
    test/misc.cpp
    test/state_tracking.cpp)
  target_compile_options(controller_common_unit_tests PRIVATE -Wno-double-promotion)
  target_link_libraries(controller_common_unit_tests ${PROJECT_NAME})
  add_dependencies(controller_common_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
