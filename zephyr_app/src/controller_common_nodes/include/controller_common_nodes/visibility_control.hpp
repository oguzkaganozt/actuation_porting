// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/controller_common_nodes/include/controller_common_nodes/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef CONTROLLER_COMMON_NODES__VISIBILITY_CONTROL_HPP_
#define CONTROLLER_COMMON_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(CONTROLLER_COMMON_NODES_BUILDING_DLL) || defined(CONTROLLER_COMMON_NODES_EXPORTS)
    #define CONTROLLER_COMMON_NODES_PUBLIC __declspec(dllexport)
    #define CONTROLLER_COMMON_NODES_LOCAL
  #else
// defined(CONTROLLER_COMMON_NODES_BUILDING_DLL) || defined(CONTROLLER_COMMON_NODES_EXPORTS)
    #define CONTROLLER_COMMON_NODES_PUBLIC __declspec(dllimport)
    #define CONTROLLER_COMMON_NODES_LOCAL
  #endif
// defined(CONTROLLER_COMMON_NODES_BUILDING_DLL) || defined(CONTROLLER_COMMON_NODES_EXPORTS)
#elif defined(__linux__)
  #define CONTROLLER_COMMON_NODES_PUBLIC __attribute__((visibility("default")))
  #define CONTROLLER_COMMON_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define CONTROLLER_COMMON_NODES_PUBLIC __attribute__((visibility("default")))
  #define CONTROLLER_COMMON_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #define CONTROLLER_COMMON_NODES_PUBLIC
  #define CONTROLLER_COMMON_NODES_LOCAL
#endif  // defined(_WINDOWS)

#endif  // CONTROLLER_COMMON_NODES__VISIBILITY_CONTROL_HPP_
