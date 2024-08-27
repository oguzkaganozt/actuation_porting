// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_geometry/include/geometry/visibility_control.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2017-2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef GEOMETRY__VISIBILITY_CONTROL_HPP_
#define GEOMETRY__VISIBILITY_CONTROL_HPP_


////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
    #define GEOMETRY_PUBLIC __declspec(dllexport)
    #define GEOMETRY_LOCAL
  #else  // defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
    #define GEOMETRY_PUBLIC __declspec(dllimport)
    #define GEOMETRY_LOCAL
  #endif  // defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
#elif defined(__linux__)
  #define GEOMETRY_PUBLIC __attribute__((visibility("default")))
  #define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GEOMETRY_PUBLIC __attribute__((visibility("default")))
  #define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#elif defined(QNX)
  #define GEOMETRY_PUBLIC __attribute__((visibility("default")))
  #define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #define GEOMETRY_PUBLIC
  #define GEOMETRY_LOCAL
#endif  // defined(__WIN32)
#endif  // GEOMETRY__VISIBILITY_CONTROL_HPP_
