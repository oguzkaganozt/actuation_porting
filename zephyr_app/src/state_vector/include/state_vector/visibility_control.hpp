// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/state_vector/include/state_vector/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef STATE_VECTOR__VISIBILITY_CONTROL_HPP_
#define STATE_VECTOR__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
    #define STATE_VECTOR_PUBLIC __declspec(dllexport)
    #define STATE_VECTOR_LOCAL
  #else  // defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
    #define STATE_VECTOR_PUBLIC __declspec(dllimport)
    #define STATE_VECTOR_LOCAL
  #endif  // defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
#elif defined(__linux__)
  #define STATE_VECTOR_PUBLIC __attribute__((visibility("default")))
  #define STATE_VECTOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define STATE_VECTOR_PUBLIC __attribute__((visibility("default")))
  #define STATE_VECTOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #define STATE_VECTOR_PUBLIC
  #define STATE_VECTOR_LOCAL
#endif  // defined(__WIN32)

#endif  // STATE_VECTOR__VISIBILITY_CONTROL_HPP_
