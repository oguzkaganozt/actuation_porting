# Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)

set(CONF_FILE "prj_actuation.conf")
# Do not overwrite the OVERLAY_CONFIG value provided by the user.
set(OVERLAY_CONFIG "prj_net.conf;${OVERLAY_CONFIG}" CACHE STRING "overlay config" FORCE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr_app)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)

# # Build CycloneDDS
include(cyclonedds_config.cmake)
create_cdds_lib(${CMAKE_CURRENT_SOURCE_DIR}/../cyclonedds)

# Get compile options from Zephyr to be passed to the Autoware packages.
zephyr_get_include_directories_for_lang_as_string(C Cincludes)
zephyr_get_include_directories_for_lang_as_string(CXX CXXincludes)
zephyr_get_system_include_directories_for_lang_as_string(C Csystem_includes)
zephyr_get_system_include_directories_for_lang_as_string(CXX CXXsystem_includes)
zephyr_get_compile_options_for_lang_as_string(C Coptions)
zephyr_get_compile_options_for_lang_as_string(CXX CXXoptions)

# Pure pursuit forwards Control Commands to host via dds. Use flag
# CONTROL_CMDS_FWD_BSD_SOCKET to override this behaviour
set(CONTROL_CMDS_FWD "dds" CACHE STRING "Control Command publish channel")
if(${CONTROL_CMDS_FWD} STREQUAL "bsd_socket")
  set(CONTROL_CMDS_FWD_FLAG "-DCONTROL_CMDS_FWD_BSD_SOCKET")
  target_compile_options(app PRIVATE "-DCONTROL_CMDS_FWD_BSD_SOCKET")
elseif(${CONTROL_CMDS_FWD} STREQUAL "dds")
  set(CONTROL_CMDS_FWD_FLAG "-DCONTROL_CMDS_FWD_DDS")
  target_compile_options(app PRIVATE "-DCONTROL_CMDS_FWD_DDS")
endif()

set(external_project_cflags "${Cincludes} ${Csystem_includes} ${Coptions} ${CONTROL_CMDS_FWD_FLAG}")
set(external_project_cxxflags "${CXXincludes} ${CXXsystem_includes} ${CXXoptions} ${CONTROL_CMDS_FWD_FLAG}")

# Build pure pursuit and its dependencies with the current Zephyr compiler and flags.
# cmake derives the C++ compiler from the path to the C compiler.
set(CMAKE_ARGS -DROS_DISTRO=humble -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install -DBUILD_TESTING=OFF -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_FLAGS=${external_project_cflags} -DCMAKE_CXX_FLAGS=${external_project_cxxflags})

include(ExternalProject)
externalproject_add(autoware_cmake_pkg
  DEPENDS zephyr_interface
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/autoware_cmake
  CMAKE_ARGS ${CMAKE_ARGS}
)
# externalproject_add(actuation_msgs_pkg
#   DEPENDS autoware_cmake_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../actuation_packages/actuation_msgs
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(autoware_auto_common_pkg
#   DEPENDS actuation_msgs_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/autoware_auto_common
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(time_utils_pkg
#   DEPENDS actuation_msgs_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/time_utils
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(autoware_auto_geometry_pkg
#   DEPENDS autoware_auto_common_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/autoware_auto_geometry
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(state_vector_pkg
#   DEPENDS autoware_auto_common_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/state_vector
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(motion_common_pkg
#   DEPENDS time_utils_pkg autoware_auto_geometry_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_common
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(motion_model_pkg
#   DEPENDS state_vector_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_model
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(controller_common_pkg
#   DEPENDS motion_common_pkg motion_model_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/controller_common
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(controller_common_nodes_pkg
#   DEPENDS controller_common_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/controller_common_nodes
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(pure_pursuit_pkg
#   DEPENDS controller_common_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit
#   CMAKE_ARGS ${CMAKE_ARGS}
# )
# externalproject_add(pure_pursuit_nodes_pkg
#   DEPENDS controller_common_nodes_pkg pure_pursuit_pkg
#   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit_nodes
#   CMAKE_ARGS ${CMAKE_ARGS}
# )

# add_dependencies(app pure_pursuit_nodes_pkg)

target_sources(app PRIVATE
  src/main.c
  include/zephyr_app.hpp
)

target_include_directories(app PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/install/include
  ${CycloneDDS_INCLUDE_DIR}
)
target_include_directories(app PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/install/lib)
# Libraries must be listed in dependency order for correct static linking.
target_link_libraries(app PRIVATE
  cdds_lib
)
