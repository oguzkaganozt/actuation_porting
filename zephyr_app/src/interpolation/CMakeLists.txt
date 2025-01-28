cmake_minimum_required(VERSION 3.14)
project(interpolation)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(interpolation SHARED
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
  src/spline_interpolation_points_2d.cpp
  src/spherical_linear_interpolation.cpp
)

ament_auto_package()
