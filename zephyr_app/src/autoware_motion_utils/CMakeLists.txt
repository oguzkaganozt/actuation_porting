cmake_minimum_required(VERSION 3.14)
project(autoware_motion_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(autoware_motion_utils SHARED
  DIRECTORY src
)

ament_auto_package()
