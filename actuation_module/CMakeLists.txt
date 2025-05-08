# Copyright (c) 2024-2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)

# PROJECT SETUP
set(CONF_FILE "prj_actuation.conf")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
find_package(Eigen3 REQUIRED)
project(actuation_module)

# INCLUDES
include(GNUInstallDirs)
include(ExternalProject)

### CYCLONEDDS ###
if(${ZEPHYR_TARGET} STREQUAL "native_sim")
  message("Building native simulator")
  target_compile_options(app PRIVATE "-DNATIVE_SIM")
else()
  message("Building cyclonedds for ${ZEPHYR_TARGET}")
  include(cyclonedds_config.cmake)
  create_cdds_lib(${CMAKE_CURRENT_SOURCE_DIR}/../cyclonedds)
endif()

### SET ZEPHYR COMPILER AND FLAGS ###
# Get compile options from Zephyr to be passed to the Autoware packages.
zephyr_get_include_directories_for_lang_as_string(C Cincludes)
zephyr_get_include_directories_for_lang_as_string(CXX CXXincludes)
zephyr_get_system_include_directories_for_lang_as_string(C Csystem_includes)
zephyr_get_system_include_directories_for_lang_as_string(CXX CXXsystem_includes)
zephyr_get_compile_options_for_lang_as_string(C Coptions)
zephyr_get_compile_options_for_lang_as_string(CXX CXXoptions)

set(external_project_cflags "${Cincludes} ${Csystem_includes} ${Coptions} -Wno-error")
set(external_project_cxxflags "${CXXincludes} ${CXXsystem_includes} ${CXXoptions} -Wno-error")

# Set current Zephyr compiler and flags.
# cmake derives the C++ compiler from the path to the C compiler.
set(CMAKE_ARGS
    "-DROS_DISTRO=humble"
    "-DCMAKE_BUILD_TYPE=Debug"
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install"
    "-DBUILD_TESTING=OFF"
    "-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY"
    "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    "-DCMAKE_C_FLAGS=${external_project_cflags}"
    "-DCMAKE_CXX_FLAGS=${external_project_cxxflags}")

### BUILD ACTUATION MESSAGES ###
externalproject_add(autoware_cmake_pkg
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../autoware_packages/autoware_cmake
  CMAKE_ARGS ${CMAKE_ARGS}
)
externalproject_add(actuation_msgs_pkg
  DEPENDS autoware_cmake_pkg
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../autoware_packages/actuation_msgs
  CMAKE_ARGS ${CMAKE_ARGS}
)
add_dependencies(app actuation_msgs_pkg)

### BUILD ACTUATION MODULE ###
set(APP_SOURCES "" CACHE INTERNAL "Global source files list")
set(APP_INCLUDE_DIRS "" CACHE INTERNAL "Global include directories list")

# Build Type
if(${BUILD_TEST} STREQUAL "1")
  target_sources(app PRIVATE test/unit_test.cpp)
elseif(${BUILD_TEST} STREQUAL "2")
  target_sources(app PRIVATE test/dds_pub.cpp)
elseif(${BUILD_TEST} STREQUAL "3")
  target_sources(app PRIVATE test/dds_sub.cpp)
else()
  # Add Autoware components
  include(src/autoware/autoware_universe_utils/autoware_universe_utils.cmake)
  include(src/autoware/autoware_interpolation/autoware_interpolation.cmake)
  include(src/autoware/autoware_motion_utils/autoware_motion_utils.cmake)
  include(src/autoware/autoware_vehicle_info_utils/autoware_vehicle_info_utils.cmake)
  include(src/autoware/autoware_trajectory_follower_base/autoware_trajectory_follower_base.cmake)
  include(src/autoware/autoware_mpc_lateral_controller/autoware_mpc_lateral_controller.cmake)
  include(src/autoware/autoware_pid_longitudinal_controller/autoware_pid_longitudinal_controller.cmake)
  include(src/autoware/autoware_trajectory_follower_node/autoware_trajectory_follower_node.cmake)
  target_sources(app PRIVATE src/main.cpp ${APP_SOURCES})
endif()

message("=== Build Summary ===")
message("App sources: ${APP_SOURCES}")
message("App headers: ${APP_INCLUDE_DIRS}")

if(${ZEPHYR_TARGET} STREQUAL "native_sim")
  set(CycloneDDS_INCLUDE_DIR "/opt/ros/humble/include")
endif()

target_include_directories(app PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/install/include
  ${CycloneDDS_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${APP_INCLUDE_DIRS}
)

# Link actuation messages and CycloneDDS library
target_link_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/install/lib)
if(${ZEPHYR_TARGET} STREQUAL "native_sim")
  target_link_libraries(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../cyclonedds_native_sim/out/lib/libddsc.a  actuation_msgs)
else()
  target_link_libraries(app PRIVATE cdds_lib actuation_msgs)
endif()
