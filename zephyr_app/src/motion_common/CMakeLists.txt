# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/CMakeLists.txt
    # In open-source project: Autoware.Auto
    # Original file: Copyright (c) 2019, Christopher Ho
    # Modifications: Copyright (c) 2023, Arm Limited.
    # SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.6)

project(motion_common)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Build library
ament_auto_add_library(${PROJECT_NAME}
  src/motion_common/config.cpp
  src/motion_common/motion_common.cpp
  src/motion_common/trajectory_common.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CycloneDDS_INCLUDE_DIR}
)
ament_export_include_directories(${actuation_msgs_INCLUDE_DIRS})

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit test
  apex_test_tools_add_gtest(motion_common_unit_tests
    test/interpolation.cpp
    test/trajectory.cpp)
  target_compile_options(motion_common_unit_tests PRIVATE -Wno-float-conversion)
  target_link_libraries(motion_common_unit_tests ${PROJECT_NAME})
  add_dependencies(motion_common_unit_tests ${PROJECT_NAME})
  ament_target_dependencies(motion_common_unit_tests
    "geometry_msgs"
  )
endif()

# Install snippets for code generation
install(DIRECTORY scripts/autogeneration_code_snippets DESTINATION share/)

ament_auto_package()
