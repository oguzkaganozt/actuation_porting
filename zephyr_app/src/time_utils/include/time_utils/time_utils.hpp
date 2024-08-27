// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/time_utils/include/time_utils/time_utils.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019-2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef TIME_UTILS__TIME_UTILS_HPP_
#define TIME_UTILS__TIME_UTILS_HPP_

#include <time_utils/visibility_control.hpp>
#include <Duration.h>
#include <Time.h>

#include <chrono>

namespace time_utils
{
/// Convert from std::chrono::time_point to time message
TIME_UTILS_PUBLIC builtin_interfaces_msg_Time to_message(std::chrono::system_clock::time_point t);
/// Convert from std::chrono::duration to duration message
TIME_UTILS_PUBLIC builtin_interfaces_msg_Duration to_message(std::chrono::nanoseconds dt);
/// Convert from std::chrono::time_point from time message
TIME_UTILS_PUBLIC
std::chrono::system_clock::time_point from_message(builtin_interfaces_msg_Time t) noexcept;
/// Convert from std::chrono::duration from duration message
TIME_UTILS_PUBLIC
std::chrono::nanoseconds from_message(builtin_interfaces_msg_Duration dt) noexcept;
/// Standard interpolation
TIME_UTILS_PUBLIC std::chrono::nanoseconds interpolate(
  std::chrono::nanoseconds a,
  std::chrono::nanoseconds b,
  float t) noexcept;

namespace details
{
template<typename TimeT>
TimeT duration_to_msg(std::chrono::nanoseconds dt);
}  // namespace details
}  // namespace time_utils

#endif  // TIME_UTILS__TIME_UTILS_HPP_
