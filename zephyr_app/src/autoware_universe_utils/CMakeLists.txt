cmake_minimum_required(VERSION 3.14)
project(autoware_universe_utils)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

find_package(fmt REQUIRED)

ament_auto_add_library(autoware_universe_utils SHARED
  src/geometry/geometry.cpp
  src/geometry/pose_deviation.cpp
  src/geometry/boost_polygon_utils.cpp
  src/geometry/random_convex_polygon.cpp
  src/geometry/gjk_2d.cpp
  src/math/sin_table.cpp
  src/math/trigonometry.cpp
  src/ros/msg_operation.cpp
  src/ros/marker_helper.cpp
  src/ros/logger_level_configure.cpp
  src/system/backtrace.cpp
  src/system/time_keeper.cpp
)

target_link_libraries(autoware_universe_utils
  fmt::fmt
)

ament_auto_package()
