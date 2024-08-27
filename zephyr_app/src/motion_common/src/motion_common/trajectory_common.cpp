// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/src/motion_common/trajectory_common.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include "motion_common/trajectory_common.hpp"

#include <limits>

#include "motion_common/motion_common.hpp"

namespace autoware
{
namespace motion
{
namespace motion_common
{
using ::motion::motion_common::to_angle;

void validateNonEmpty(const Points & points)
{
  if (points._length == 0) {
    throw std::invalid_argument("Empty points");
  }
}

float64_t calcYawDeviation(const float64_t & base_yaw, const float64_t & target_yaw)
{
  return autoware::common::helper_functions::wrap_angle(target_yaw - base_yaw);
}

std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points, const size_t src_idx, const size_t dst_idx, const float64_t epsilon)
{
  validateNonEmpty(points);

  if (dst_idx >= points._length) {
    throw std::out_of_range{"points access out of range"};
  }
  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (static_cast<float64_t>(std::fabs(points._buffer[i].longitudinal_velocity_mps)) < epsilon) {
      return i;
    }
  }

  return {};
}

std::experimental::optional<size_t> searchZeroVelocityIndex(const Points & points)
{
  return searchZeroVelocityIndex(points, 0, points._length);
}

std::experimental::optional<size_t> findNearestIndex(
  const Points & points, const geometry_msgs_msg_Pose & pose,
  const float64_t max_dist,
  const float64_t max_yaw)
{
  validateNonEmpty(points);

  float64_t min_dist = std::numeric_limits<float64_t>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  const auto target_yaw = to_angle(pose.orientation);
  for (size_t i = 0; i < points._length; ++i) {
    const auto dist =
      autoware::common::geometry::distance_2d<float64_t>(points._buffer[i], pose.position);
    if (dist > max_dist) {
      continue;
    }

    const auto base_yaw = to_angle(points._buffer[i].pose.orientation);
    const auto yaw = calcYawDeviation(base_yaw, target_yaw);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    if (dist >= min_dist) {
      continue;
    }

    min_dist = dist;
    min_idx = i;
    is_nearest_found = true;
  }
  return is_nearest_found ? std::experimental::optional<size_t>(min_idx) : std::experimental::
         nullopt;
}

float64_t calcSignedArcLength(const Points & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  float64_t dist_sum = 0.0;
  if (dst_idx + 1 >= points._length) {
    throw std::out_of_range{"points access out of range"};
  }
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += autoware::common::geometry::distance_2d<float64_t>(
      points._buffer[i],
      points._buffer[i + 1]);
  }
  return dist_sum;
}
}  // namespace motion_common
}  // namespace motion
}  // namespace autoware
