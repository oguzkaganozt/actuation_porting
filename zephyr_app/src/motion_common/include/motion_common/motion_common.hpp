// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/include/motion_common/motion_common.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019-2021, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
#ifndef MOTION_COMMON__MOTION_COMMON_HPP_
#define MOTION_COMMON__MOTION_COMMON_HPP_

#include <motion_common/visibility_control.hpp>
#include <Trajectory.h>
#include <VehicleControlCommand.h>
#include <VehicleKinematicState.h>

#include <time_utils/time_utils.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion
{
namespace motion_common
{
// Use same representation as message type
using Real = decltype(autoware_auto_planning_msgs_msg_TrajectoryPoint::longitudinal_velocity_mps);
using Command = autoware_auto_vehicle_msgs_msg_VehicleControlCommand;
using State = autoware_auto_vehicle_msgs_msg_VehicleKinematicState;
using Trajectory = autoware_auto_planning_msgs_msg_Trajectory;
using Orientation = geometry_msgs_msg_Quaternion;
using Double = decltype(Orientation::x);
using Index = size_t;
using Point = autoware_auto_planning_msgs_msg_TrajectoryPoint;

/// Check if a state is past a given trajectory point, assuming heading is correct
MOTION_COMMON_PUBLIC bool is_past_point(const Point & state, const Point & pt) noexcept;
/// Check if a state is past a given trajectory point given the last (aka current)
/// trajectory point
MOTION_COMMON_PUBLIC bool is_past_point(
  const Point & state, const Point & current_pt, const Point & next_pt) noexcept;
/// Given a normal, determine if state is past a point
MOTION_COMMON_PUBLIC
bool is_past_point(const Point & state, const Point & pt, Double nx, Double ny) noexcept;

/// Advance to the first trajectory point past state according to criterion is_past_point
template<typename IsPastPointF>
Index update_reference_index(
  const Trajectory & traj,
  const State & state,
  Index start_idx,
  IsPastPointF is_past_point)
{
  // Invariant: m_reference_trajectory.points.size > 0U
  if (traj.points._length == 0) {
    return 0U;
  }
  auto next_idx = start_idx;
  for (auto idx = start_idx; idx < traj.points._length - 1U; ++idx) {
    const auto current_pt = traj.points._buffer[idx];
    const auto next_pt = traj.points._buffer[idx + 1U];

    // If I'm not past the next point, then I'm done
    if (!is_past_point(state, current_pt, next_pt, traj)) {
      break;
    }
    // Otherwise, update
    next_idx = idx + 1U;
  }
  return next_idx;
}

/// Check that all heading values in a trajectory are normalized 2D quaternions
MOTION_COMMON_PUBLIC bool heading_ok(const Trajectory & traj);

////////////////////////////////////////////////////////////////////////////////
// Operations relating to algebraic manipulation of VehicleKinematicState and TrajectoryPoint

/// Converts 3D quaternion to simple heading representation
MOTION_COMMON_PUBLIC Double to_angle(Orientation orientation) noexcept;

/// \brief Converts angles into a corresponding Orientation
/// \tparam RealT a floating point type
/// \param[in] roll angle to use as roll of the Orientation [radians]
/// \param[in] pitch angle to use as pitch of the Orientation [radians]
/// \param[in] yaw angle to use as yaw of the Orientation [radians]
/// \returns A converted Orientation object
template<typename RealT>
Orientation from_angles(RealT roll, RealT pitch, RealT yaw) noexcept
{
  static_assert(std::is_floating_point<RealT>::value, "angle must be floating point");

  // From ROS2 tf2::Quaternion::setRPY()
  Double halfYaw = static_cast<Double>(yaw) * 0.5;
  Double halfPitch = static_cast<Double>(pitch) * 0.5;
  Double halfRoll = static_cast<Double>(roll) * 0.5;
  Double cosYaw = std::cos(halfYaw);
  Double sinYaw = std::sin(halfYaw);
  Double cosPitch = std::cos(halfPitch);
  Double sinPitch = std::sin(halfPitch);
  Double cosRoll = std::cos(halfRoll);
  Double sinRoll = std::sin(halfRoll);
  Orientation quat {
    sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
    cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
    cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
    cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw //formerly yzx
  };

  return quat;
}

/// \brief Converts a heading angle into a corresponding Orientation
/// \tparam RealT a floating point type
/// \param[in] angle heading angle to use as yaw of the Orientation [radians]
/// \returns A converted Orientation object
template<typename RealT>
Orientation from_angle(RealT angle) noexcept
{
  return from_angles(RealT{}, RealT{}, angle);
}

/// Standard clamp implementation
template<typename T>
T clamp(T val, T min, T max)
{
  if (max < min) {
    throw std::domain_error{"max < min"};
  }
  return std::min(max, std::max(min, val));
}

/// Standard linear interpolation
template<typename T, typename RealT = double>
T interpolate(T a, T b, RealT t)
{
  static_assert(std::is_floating_point<RealT>::value, "t must be floating point");
  t = clamp(t, RealT{0.0}, RealT{1.0});
  const auto del = b - a;
  return static_cast<T>(t * static_cast<RealT>(del)) + a;
}

/// Spherical linear interpolation
MOTION_COMMON_PUBLIC Orientation slerp(const Orientation & a, const Orientation & b, const Real t);

/// Trajectory point interpolation
template<typename SlerpF>
Point interpolate(Point a, Point b, Real t, SlerpF slerp_fn)
{
  Point ret{};
  {
    const auto dt0 = time_utils::from_message(a.time_from_start);
    const auto dt1 = time_utils::from_message(b.time_from_start);
    ret.time_from_start = time_utils::to_message(time_utils::interpolate(dt0, dt1, t));
  }
  ret.pose.position.x = interpolate(a.pose.position.x, b.pose.position.x, t);
  ret.pose.position.y = interpolate(a.pose.position.y, b.pose.position.y, t);
  ret.pose.orientation = slerp_fn(a.pose.orientation, b.pose.orientation, t);
  ret.longitudinal_velocity_mps =
    interpolate(a.longitudinal_velocity_mps, b.longitudinal_velocity_mps, t);
  ret.lateral_velocity_mps = interpolate(a.lateral_velocity_mps, b.lateral_velocity_mps, t);
  ret.acceleration_mps2 = interpolate(a.acceleration_mps2, b.acceleration_mps2, t);
  ret.heading_rate_rps = interpolate(a.heading_rate_rps, b.heading_rate_rps, t);
  ret.front_wheel_angle_rad = interpolate(a.front_wheel_angle_rad, b.front_wheel_angle_rad, t);
  ret.rear_wheel_angle_rad = interpolate(a.rear_wheel_angle_rad, b.rear_wheel_angle_rad, t);

  return ret;
}

/// Default point interpolation
MOTION_COMMON_PUBLIC Point interpolate(Point a, Point b, Real t);

/// Sample a trajectory using interpolation; does not extrapolate
template<typename SlerpF>
void sample(
  const Trajectory & in,
  Trajectory & out,
  std::chrono::nanoseconds period,
  SlerpF slerp_fn)
{
  out.points._length = 0;
  out.header = in.header;
  if (in.points._length == 0) {
    return;
  }
  // Determine number of points
  using time_utils::from_message;
  const auto last_time = from_message(in.points._buffer[in.points._length - 1].time_from_start);
  auto count_ = last_time / period;
  // should round down
  using SizeT = size_t;
  const auto count = static_cast<SizeT>(count_);
  if (out.points._maximum <= count) {
    throw std::domain_error{"points overflow"};
  }
  // Insert first point
  if (out.points._length >= out.points._maximum) {
    throw std::length_error{"points full"};
  }
  out.points._buffer[out.points._length++] = in.points._buffer[0];
  // Add remaining points
  auto ref_duration = period;
  auto after_current_ref_idx = SizeT{1};
  for (auto idx = SizeT{1}; idx < count; ++idx) {
    // Determine next reference index
    for (auto jdx = after_current_ref_idx; jdx < in.points._length; ++jdx) {
      const auto & pt = in.points._buffer[jdx];
      if (from_message(pt.time_from_start) > ref_duration) {
        after_current_ref_idx = jdx;
        break;
      }
    }
    // Interpolate
    {
      const auto & curr_pt = in.points._buffer[after_current_ref_idx - 1U];
      const auto & next_pt = in.points._buffer[after_current_ref_idx];
      const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(
        from_message(next_pt.time_from_start) - from_message(curr_pt.time_from_start));
      const auto dt_ = std::chrono::duration_cast<std::chrono::duration<Real>>(
        ref_duration - from_message(curr_pt.time_from_start));
      const auto t = dt_.count() / dt.count();
      if (out.points._length >= out.points._maximum) {
        throw std::length_error{"out.points full"};
      }
      out.points._buffer[out.points._length++] = interpolate(curr_pt, next_pt, t, slerp_fn);
    }
    ref_duration += period;
  }
}

/// Trajectory sampling with default interpolation
MOTION_COMMON_PUBLIC void sample(
  const Trajectory & in,
  Trajectory & out,
  std::chrono::nanoseconds period);
}  // namespace motion_common
}  // namespace motion
#endif  // MOTION_COMMON__MOTION_COMMON_HPP_
