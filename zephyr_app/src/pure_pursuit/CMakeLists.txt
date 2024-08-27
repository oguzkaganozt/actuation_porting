# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit/CMakeLists.txt
    # In open-source project: Autoware.Auto
    # Original file: Copyright (c) 2019, the Autoware Foundation
    # Modifications: Copyright (c) 2022-2023, Arm Limited.
    # SPDX-License-Identifier: Apache-2.0
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  include/pure_pursuit/config.hpp
  src/config.cpp
  include/pure_pursuit/pure_pursuit.hpp
  src/pure_pursuit.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CycloneDDS_INCLUDE_DIR}
)

# Setting a system include is needed to avoid having an explicit Eigen dependency.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(PURE_PURSUIT_GTEST pure_pursuit_gtest)
  ament_add_gtest(${PURE_PURSUIT_GTEST}
    test/gtest_main.cpp
    test/sanity_checks.cpp
    test/test_pure_pursuit.cpp)
  target_include_directories(${PURE_PURSUIT_GTEST} PRIVATE "include")
  target_link_libraries(${PURE_PURSUIT_GTEST} ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools)
  ament_target_dependencies(${PURE_PURSUIT_GTEST} "osrf_testing_tools_cpp")
endif()

# Ament Exporting
ament_auto_package()
