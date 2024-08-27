# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/time_utils/CMakeLists.txt
# In open-source project: Autoware.Auto
# Original file: Copyright (c) 2019, Christopher Ho
# Modifications: Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.6)

project(time_utils)

include(../../cyclonedds_config.cmake)
cyclonedds_include(CycloneDDS_INCLUDE_DIR)

### Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

### Build
ament_auto_add_library(${PROJECT_NAME} src/time_utils/time_utils.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CycloneDDS_INCLUDE_DIR}
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # TODO(c.ho) unit tests
endif()

ament_auto_package()
