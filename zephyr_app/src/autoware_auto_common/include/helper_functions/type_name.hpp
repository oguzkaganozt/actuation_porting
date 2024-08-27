// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/type_name.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef HELPER_FUNCTIONS__TYPE_NAME_HPP_
#define HELPER_FUNCTIONS__TYPE_NAME_HPP_

#include <common/visibility_control.hpp>

#include <string>
#include <typeinfo>

#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
#include <cxxabi.h>
#endif

namespace autoware
{
namespace helper_functions
{

/// @brief      Get a demangled name of a type.
template<typename T>
COMMON_PUBLIC std::string get_type_name()
{
  #if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
  return abi::__cxa_demangle(typeid(T).name(), NULL, NULL, 0);
  #else
  // For unsupported compilers return a mangled name.
  return typeid(T).name();
  #endif
}

/// @brief      Get a demangled name of a type given its instance.
template<typename T>
COMMON_PUBLIC std::string get_type_name(const T &)
{
  return get_type_name<T>();
}

}  // namespace helper_functions
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__TYPE_NAME_HPP_
