# Copyright (c) 2024-2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)

# Do not overwrite the OVERLAY_CONFIG value provided by the user.
set(CONF_FILE "prj_actuation.conf")
set(OVERLAY_CONFIG "prj_net.conf;${OVERLAY_CONFIG}" CACHE STRING "overlay config" FORCE)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(actuation_autoware)

include(GNUInstallDirs)

### BUILD CYCLONEDDS ###
message(STATUS "Building CycloneDDS")
include(cyclonedds_config.cmake)
create_cdds_lib(${CMAKE_CURRENT_SOURCE_DIR}/../cyclonedds)

### SET ZEPHYR COMPILER AND FLAGS ###
message(STATUS "Setting Zephyr compiler and flags")

# Get compile options from Zephyr to be passed to the Autoware packages.
zephyr_get_include_directories_for_lang_as_string(C Cincludes)
zephyr_get_include_directories_for_lang_as_string(CXX CXXincludes)
zephyr_get_system_include_directories_for_lang_as_string(C Csystem_includes)
zephyr_get_system_include_directories_for_lang_as_string(CXX CXXsystem_includes)
zephyr_get_compile_options_for_lang_as_string(C Coptions)
zephyr_get_compile_options_for_lang_as_string(CXX CXXoptions)

# Actuation forwards Control Commands to host via dds. Use flag
# CONTROL_CMDS_FWD_BSD_SOCKET to override this behaviour
set(CONTROL_CMDS_FWD "dds" CACHE STRING "Control Command publish channel")
if(${CONTROL_CMDS_FWD} STREQUAL "bsd_socket")
  set(CONTROL_CMDS_FWD_FLAG "-DCONTROL_CMDS_FWD_BSD_SOCKET")
  target_compile_options(app PRIVATE "-DCONTROL_CMDS_FWD_BSD_SOCKET")
elseif(${CONTROL_CMDS_FWD} STREQUAL "dds")
  set(CONTROL_CMDS_FWD_FLAG "-DCONTROL_CMDS_FWD_DDS")
  target_compile_options(app PRIVATE "-DCONTROL_CMDS_FWD_DDS")
endif()

set(external_project_cflags "${Cincludes} ${Csystem_includes} ${Coptions} -Wno-error ${CONTROL_CMDS_FWD_FLAG} -Wno-error")
set(external_project_cxxflags "${CXXincludes} ${CXXsystem_includes} ${CXXoptions} -Wno-error ${CONTROL_CMDS_FWD_FLAG} -Wno-error")

# Set current Zephyr compiler and flags.
# cmake derives the C++ compiler from the path to the C compiler.
set(CMAKE_ARGS
    "-DROS_DISTRO=humble"
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install"
    "-DBUILD_TESTING=OFF"
    "-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY"
    "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    "-DCMAKE_C_FLAGS=${external_project_cflags}"
    "-DCMAKE_CXX_FLAGS=${external_project_cxxflags}"
    "-DCMAKE_CXX_FLAGS=-Wno-error")

### BUILD ACTUATION MSGS ###
message(STATUS "Building actuation_msgs")

include(ExternalProject)
externalproject_add(autoware_cmake_pkg
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../actuation_packages/autoware_cmake
  CMAKE_ARGS ${CMAKE_ARGS}
)
externalproject_add(actuation_msgs_pkg
  DEPENDS autoware_cmake_pkg
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../actuation_packages/actuation_msgs
  CMAKE_ARGS ${CMAKE_ARGS}
)
add_dependencies(app actuation_msgs_pkg)

### BUILD ACTUATION MODULES ###
message(STATUS "Building actuation modules")
set(APP_SOURCES "" CACHE INTERNAL "Global source files list")
set(APP_INCLUDE_DIRS "" CACHE INTERNAL "Global include directories list")

# Include actuation modules
include(src/autoware_osqp_interface/osqp_interface.cmake)
# include(src/autoware_universe_utils/autoware_universe_utils.cmake)
# include(src/autoware_vehicle_info_utils/autoware_vehicle_info_utils.cmake)
# include(src/interpolation/interpolation.cmake)
# include(src/autoware_trajectory_follower_base/autoware_trajectory_follower_base.cmake)
# include(src/autoware_mpc_lateral_controller/autoware_mpc_lateral_controller.cmake)
# include(src/autoware_motion_utils/autoware_motion_utils.cmake)

# Add main sources and headers
list(APPEND APP_SOURCES
  src/file.c
  src/main.c
  include/zephyr_app.hpp
)
list(APPEND APP_INCLUDE_DIRS
  ${CMAKE_CURRENT_BINARY_DIR}/install/include
  ${CycloneDDS_INCLUDE_DIR}
)

# Add actuation modules
add_osqp_interface()
# add_autoware_universe_utils(app_sources app_headers)
# add_autoware_vehicle_info_utils(app_sources app_headers)
# add_interpolation(app_sources app_headers)
# add_autoware_trajectory_follower_base(app_sources app_headers)
# add_autoware_mpc_lateral_controller(app_sources app_headers)
# add_autoware_motion_utils(app_sources app_headers)

# Build
message(STATUS "app_sources: ${APP_SOURCES}")
message(STATUS "app_headers: ${APP_INCLUDE_DIRS}")
target_sources(app PRIVATE
  ${APP_SOURCES}
)

target_include_directories(app PRIVATE
  include
  ${APP_INCLUDE_DIRS}
)

# target_link_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/install/lib)