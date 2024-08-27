// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/time_utils/include/time_utils/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef TIME_UTILS__VISIBILITY_CONTROL_HPP_
#define TIME_UTILS__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(TIME_UTILS_BUILDING_DLL) || defined(TIME_UTILS_EXPORTS)
    #define TIME_UTILS_PUBLIC __declspec(dllexport)
    #define TIME_UTILS_LOCAL
  #else  // defined(TIME_UTILS_BUILDING_DLL) || defined(TIME_UTILS_EXPORTS)
    #define TIME_UTILS_PUBLIC __declspec(dllimport)
    #define TIME_UTILS_LOCAL
  #endif  // defined(TIME_UTILS_BUILDING_DLL) || defined(TIME_UTILS_EXPORTS)
#elif defined(__linux__)
  #define TIME_UTILS_PUBLIC __attribute__((visibility("default")))
  #define TIME_UTILS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define TIME_UTILS_PUBLIC __attribute__((visibility("default")))
  #define TIME_UTILS_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #define TIME_UTILS_PUBLIC
  #define TIME_UTILS_LOCAL
#endif  // defined(_WINDOWS)

#endif  // TIME_UTILS__VISIBILITY_CONTROL_HPP_
