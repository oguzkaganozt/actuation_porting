// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit/include/pure_pursuit/config.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef PURE_PURSUIT__CONFIG_HPP_
#define PURE_PURSUIT__CONFIG_HPP_

#include <pure_pursuit/visibility_control.hpp>
#include <common/types.hpp>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace control
{
namespace pure_pursuit
{
/// \brief A configuration class for the PurePursuit class.
class PURE_PURSUIT_PUBLIC Config
{
public:
  /// \brief Constructor
  /// \param[in] minimum_lookahead_distance The minimum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] maximum_lookahead_distance The maximum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] speed_to_lookahead_ratio The conversion ratio from the speed to
  ///            the lookahead distance. The ratio is equal to the duration (s)
  /// \param[in] is_interpolate_lookahead_point The boolean whether using the interpolation
  ///            for determining the target position
  /// \param[in] is_delay_compensation The boolean whethre using the delay compensation
  ///            for estimating the current vehicle position at the current timestamp
  /// \param[in] emergency_stop_distance The emergency stop distance for the emergency stop
  /// \param[in] speed_thres_traveling_direction The speed threshold for
  ///            determining the traveling direction
  /// \param[in] distance_front_rear_wheel The distance between front and rear wheels
  Config(
    const float32_t minimum_lookahead_distance,
    const float32_t maximum_lookahead_distance,
    const float32_t speed_to_lookahead_ratio,
    const bool8_t is_interpolate_lookahead_point,
    const bool8_t is_delay_compensation,
    const float32_t emergency_stop_distance,
    const float32_t speed_thres_traveling_direction,
    const float32_t distance_front_rear_wheel);
  /// \brief Gets the minimum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_minimum_lookahead_distance() const noexcept;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_maximum_lookahead_distance() const noexcept;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_speed_to_lookahead_ratio() const noexcept;
  /// \brief Gets the boolean whether using the interpolation to get the target point
  /// \return Fixed value
  bool8_t get_is_interpolate_lookahead_point() const noexcept;
  /// \brief Gets the boolean whether using the delay compensation to estimate the current point
  /// \return Fixed value
  bool8_t get_is_delay_compensation() const noexcept;
  /// \brief Gets the emergency target distance for the emergency stop
  /// \return Fixed value
  float32_t get_emergency_stop_distance() const noexcept;
  /// \brief Gets the speed threshold for determining the traveling direction
  /// \return Fixed value
  float32_t get_speed_thres_traveling_direction() const noexcept;
  /// \brief Gets the distance between front and rear wheels
  /// \return Fixed value
  float32_t get_distance_front_rear_wheel() const noexcept;

private:
  float32_t m_minimum_lookahead_distance;
  float32_t m_maximum_lookahead_distance;
  float32_t m_speed_to_lookahead_ratio;
  bool8_t m_is_interpolate_lookahead_point;
  bool8_t m_is_delay_compensation;
  float32_t m_emergency_stop_distance;
  float32_t m_speed_thres_traveling_direction;
  float32_t m_distance_front_rear_wheel;
};  // class Config
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT__CONFIG_HPP_
