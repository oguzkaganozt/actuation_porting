# Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_geometry/CMakeLists.txt
    # In open-source project: Autoware.Auto
    # Original file: Copyright (c) 2019, the Autoware Foundation
    # Modifications: Copyright (c) 2022-2023, Arm Limited.
    # SPDX-License-Identifier: Apache-2.0
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_geometry)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # gtest
  set(GEOMETRY_GTEST geometry_gtest)
  set(GEOMETRY_SRC test/src/test_geometry.cpp
    test/src/test_convex_hull.cpp
    test/src/test_hull_pockets.cpp
    test/src/test_interval.cpp
    test/src/lookup_table.cpp
    test/src/test_area.cpp
    test/src/test_common_2d.cpp
    test/src/test_intersection.cpp
  )
  ament_add_gtest(${GEOMETRY_GTEST} ${GEOMETRY_SRC})
  target_compile_options(${GEOMETRY_GTEST} PRIVATE -Wno-conversion -Wno-sign-conversion)
  target_include_directories(${GEOMETRY_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${GEOMETRY_GTEST}
    "autoware_auto_common"
    "autoware_auto_perception_msgs"
    "autoware_auto_planning_msgs"
    "autoware_auto_vehicle_msgs"
    "geometry_msgs"
    "osrf_testing_tools_cpp")
  target_link_libraries(${GEOMETRY_GTEST} ${PROJECT_NAME})
endif()

# Ament Exporting
ament_auto_package()
