// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit/src/config.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include <limits>
#include <stdexcept>
#include "pure_pursuit/config.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace pure_pursuit
{
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const float32_t minimum_lookahead_distance,
  const float32_t maximum_lookahead_distance,
  const float32_t speed_to_lookahead_ratio,
  const bool8_t is_interpolate_lookahead_point,
  const bool8_t is_delay_compensation,
  const float32_t emergency_stop_distance,
  const float32_t speed_thres_traveling_direction,
  const float32_t distance_front_rear_wheel)
: m_minimum_lookahead_distance(minimum_lookahead_distance),
  m_maximum_lookahead_distance(maximum_lookahead_distance),
  m_speed_to_lookahead_ratio(speed_to_lookahead_ratio),
  m_is_interpolate_lookahead_point(is_interpolate_lookahead_point),
  m_is_delay_compensation(is_delay_compensation),
  m_emergency_stop_distance(emergency_stop_distance),
  m_speed_thres_traveling_direction(speed_thres_traveling_direction),
  m_distance_front_rear_wheel(distance_front_rear_wheel)
{
  if (m_minimum_lookahead_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config minimum lookahead distance is lower than 0");
  }
  if (m_maximum_lookahead_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config maximum lookahead distance is lower than 0");
  }
  if (m_speed_to_lookahead_ratio <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config: speed to lookahead ratio is lower than 0");
  }
  if (emergency_stop_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config: emergency_stop_distance is lower than 0");
  }
  if (m_speed_thres_traveling_direction < 0.0F) {
    throw std::domain_error(
            "pure_pursuit::Config: speed_thres_traveling_direction is lower than 0");
  }
  if (m_distance_front_rear_wheel < 0.0F) {
    throw std::domain_error(
            "pure_pursuit::Config: distance_front_rear_wheel is lower than 0");
  }
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_minimum_lookahead_distance() const noexcept
{
  return m_minimum_lookahead_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_maximum_lookahead_distance() const noexcept
{
  return m_maximum_lookahead_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_speed_to_lookahead_ratio() const noexcept
{
  return m_speed_to_lookahead_ratio;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t Config::get_is_interpolate_lookahead_point() const noexcept
{
  return m_is_interpolate_lookahead_point;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t Config::get_is_delay_compensation() const noexcept
{
  return m_is_delay_compensation;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_emergency_stop_distance() const noexcept
{
  return m_emergency_stop_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_speed_thres_traveling_direction() const noexcept
{
  return m_speed_thres_traveling_direction;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_distance_front_rear_wheel() const noexcept
{
  return m_distance_front_rear_wheel;
}
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware
