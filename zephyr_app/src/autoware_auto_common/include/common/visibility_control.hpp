// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/common/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2017-2019, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef COMMON__VISIBILITY_CONTROL_HPP_
#define COMMON__VISIBILITY_CONTROL_HPP_

#if defined(_MSC_VER) && defined(_WIN64)
  #if defined(COMMON_BUILDING_DLL) || defined(COMMON_EXPORTS)
    #define COMMON_PUBLIC __declspec(dllexport)
    #define COMMON_LOCAL
  #else  // defined(COMMON_BUILDING_DLL) || defined(COMMON_EXPORTS)
    #define COMMON_PUBLIC __declspec(dllimport)
    #define COMMON_LOCAL
  #endif  // defined(COMMON_BUILDING_DLL) || defined(COMMON_EXPORTS)
#elif defined(__GNUC__) && defined(__linux__)
  #define COMMON_PUBLIC __attribute__((visibility("default")))
  #define COMMON_LOCAL __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && defined(__APPLE__)
  #define COMMON_PUBLIC __attribute__((visibility("default")))
  #define COMMON_LOCAL __attribute__((visibility("hidden")))
#else  // !(defined(__GNUC__) && defined(__APPLE__))
  #define COMMON_PUBLIC
  #define COMMON_LOCAL
#endif  // _MSC_VER

#endif  // COMMON__VISIBILITY_CONTROL_HPP_
