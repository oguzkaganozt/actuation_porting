// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/include/motion_common/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTION_COMMON__VISIBILITY_CONTROL_HPP_
#define MOTION_COMMON__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
    #define MOTION_COMMON_PUBLIC __declspec(dllexport)
    #define MOTION_COMMON_LOCAL
  #else  // defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
    #define MOTION_COMMON_PUBLIC __declspec(dllimport)
    #define MOTION_COMMON_LOCAL
  #endif  // defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
#elif defined(__linux__)
  #define MOTION_COMMON_PUBLIC __attribute__((visibility("default")))
  #define MOTION_COMMON_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MOTION_COMMON_PUBLIC __attribute__((visibility("default")))
  #define MOTION_COMMON_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #define MOTION_COMMON_PUBLIC
  #define MOTION_COMMON_LOCAL
#endif  // defined(_WINDOWS)

#endif  // MOTION_COMMON__VISIBILITY_CONTROL_HPP_
