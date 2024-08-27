// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/motion_model/include/motion_model/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__VISIBILITY_CONTROL_HPP_
#define MOTION_MODEL__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
    #define MOTION_MODEL_PUBLIC __declspec(dllexport)
    #define MOTION_MODEL_LOCAL
  #else  // defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
    #define MOTION_MODEL_PUBLIC __declspec(dllimport)
    #define MOTION_MODEL_LOCAL
  #endif  // defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
#elif defined(__linux__)
  #define MOTION_MODEL_PUBLIC __attribute__((visibility("default")))
  #define MOTION_MODEL_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MOTION_MODEL_PUBLIC __attribute__((visibility("default")))
  #define MOTION_MODEL_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #define MOTION_MODEL_PUBLIC
  #define MOTION_MODEL_LOCAL
#endif  // defined(__WIN32)

#endif  // MOTION_MODEL__VISIBILITY_CONTROL_HPP_
