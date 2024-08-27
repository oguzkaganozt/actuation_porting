# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/state_vector/CMakeLists.txt
# In open-source project: Autoware.Auto
# Original file: Copyright (c) 2021, The Autoware Foundation
# Modifications: Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)
project(state_vector)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# build library
ament_auto_add_library(
  ${PROJECT_NAME}
  src/common_states.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  set(STATE_VECTOR_GTEST state_vector_gtest)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  ament_add_gtest(${STATE_VECTOR_GTEST}
                  test/test_variables.cpp
                  test/test_generic_state.cpp)
  target_include_directories(${STATE_VECTOR_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${STATE_VECTOR_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${STATE_VECTOR_GTEST} "autoware_auto_common")
endif()

# Ament Exporting
ament_export_include_directories(${EIGEN3_INCLUDE_DIR})
ament_auto_package()
