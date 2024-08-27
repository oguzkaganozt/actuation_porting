// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit/include/pure_pursuit/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef PURE_PURSUIT__VISIBILITY_CONTROL_HPP_
#define PURE_PURSUIT__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
    #define PURE_PURSUIT_PUBLIC __declspec(dllexport)
    #define PURE_PURSUIT_LOCAL
  #else  // defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
    #define PURE_PURSUIT_PUBLIC __declspec(dllimport)
    #define PURE_PURSUIT_LOCAL
  #endif  // defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
#elif defined(__linux__)
  #define PURE_PURSUIT_PUBLIC __attribute__((visibility("default")))
  #define PURE_PURSUIT_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define PURE_PURSUIT_PUBLIC __attribute__((visibility("default")))
  #define PURE_PURSUIT_LOCAL __attribute__((visibility("hidden")))
#else  // defined(LINUX)
  #define PURE_PURSUIT_PUBLIC
  #define PURE_PURSUIT_LOCAL
#endif  // defined(WINDOWS)

#endif  // PURE_PURSUIT__VISIBILITY_CONTROL_HPP_
