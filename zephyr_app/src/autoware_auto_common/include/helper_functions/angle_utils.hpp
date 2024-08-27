// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/angle_utils.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2020, Apex.AI, Inc.
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef HELPER_FUNCTIONS__ANGLE_UTILS_HPP_
#define HELPER_FUNCTIONS__ANGLE_UTILS_HPP_

#include <cmath>
#include <type_traits>

// M_PI is not declared in newlib's headers
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

namespace autoware
{
namespace common
{
namespace helper_functions
{

namespace detail
{
constexpr auto kDoublePi = 2.0 * M_PI;
}  // namespace detail

///
/// @brief      Wrap angle to the [-pi, pi] range.
///
/// @details    This method uses the formula suggested in the paper [On wrapping the Kalman filter
///             and estimating with the SO(2) group](https://arxiv.org/pdf/1708.05551.pdf) and
///             implements the following formula:
///             \f$\mathrm{mod}(\alpha + \pi, 2 \pi) - \pi\f$.
///
/// @param[in]  angle  The input angle
///
/// @tparam     T      Type of scalar
///
/// @return     Angle wrapped to the chosen range.
///
template<typename T>
constexpr T wrap_angle(T angle) noexcept
{
  auto help_angle = angle + T(M_PI);
  while (help_angle < T{}) {help_angle += T(detail::kDoublePi);}
  while (help_angle >= T(detail::kDoublePi)) {help_angle -= T(detail::kDoublePi);}
  return help_angle - T(M_PI);
}

}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__ANGLE_UTILS_HPP_
