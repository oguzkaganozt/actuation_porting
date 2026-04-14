# Copyright (c) 2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0
#
# Shared source and include path definitions for the actuation module.
# Included by both Zephyr and FreeRTOS CMake builds.

set(ACTUATION_MODULE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(AUTOWARE_COMPONENTS
    autoware_universe_utils
    autoware_interpolation
    autoware_motion_utils
    autoware_vehicle_info_utils
    autoware_trajectory_follower_base
    autoware_mpc_lateral_controller
    autoware_pid_longitudinal_controller
    autoware_trajectory_follower_node
)

set(APP_SOURCES "")
set(APP_INCLUDE_DIRS "")

foreach(component IN LISTS AUTOWARE_COMPONENTS)
    include(${ACTUATION_MODULE_DIR}/src/autoware/${component}/${component}.cmake)
endforeach()
