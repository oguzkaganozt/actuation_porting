// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/src/motion_common/motion_common.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019-2021, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include "motion_common/motion_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "helper_functions/angle_utils.hpp"

namespace motion
{
namespace motion_common
{
////////////////////////////////////////////////////////////////////////////////
bool is_past_point(const Point & state, const Point & pt) noexcept
{
  const auto w = pt.pose.orientation.w;
  const auto z = pt.pose.orientation.z;
  // double angle rule
  const auto c = (w + z) * (w - z);
  const auto s = 2.0 * w * z;

  return is_past_point(state, pt, c, s);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(
  const Point & state,
  const Point & current_pt,
  const Point & next_pt) noexcept
{
  const auto nx = next_pt.pose.position.x - current_pt.pose.position.x;
  const auto ny = next_pt.pose.position.y - current_pt.pose.position.y;

  return is_past_point(state, next_pt, nx, ny);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(
  const Point & state,
  const Point & pt,
  const Double nx,
  const Double ny) noexcept
{
  const auto dx = (state.pose.position.x - pt.pose.position.x);
  const auto dy = (state.pose.position.y - pt.pose.position.y);

  // Check if state is past last_pt when projected onto the ray defined by heading
  return ((nx * dx) + (ny * dy)) >= -std::numeric_limits<Double>::epsilon();
}

////////////////////////////////////////////////////////////////////////////////
bool heading_ok(const Trajectory & traj)
{
  const auto bad_heading = [](const auto & pt) -> bool {
      const auto real2 = static_cast<Real>(pt.pose.orientation.w * pt.pose.orientation.w);
      const auto imag2 = static_cast<Real>(pt.pose.orientation.z * pt.pose.orientation.z);
      constexpr auto TOL = 1.0E-3F;
      return std::fabs(1.0F - (real2 + imag2)) > TOL;
    };
  for (size_t i = 0; i < traj.points._length; i++) {
    if (bad_heading(traj.points._buffer[i])) {
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// From ROS2 tf2 utils
double getYaw(const Orientation & q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw = -2 * std::atan2(q.y, q.x);
  } else if (sarg >= 0.99999) {
    yaw = 2 * std::atan2(q.y, q.x);
  } else {
    yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

Double to_angle(Orientation orientation) noexcept
{
  return getYaw(orientation);
}

////////////////////////////////////////////////////////////////////////////////
// From ROS2 tf2::Quaternion::dot()
Double dot(const Orientation & a, const Orientation & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// From ROS2 tf2::Quaternion::angleShortestPath()
Double angleShortestPath(const Orientation & a, const Orientation & b)
{
  Double s = std::sqrt(dot(a, a) * dot(b, b));
  if (dot(a, b) < 0) { // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
    return std::acos(clamp(-dot(a, b) / s, static_cast<Double>(-1), static_cast<Double>(1))) * 2;
  } else {
    return std::acos(clamp(dot(a, b) / s, static_cast<Double>(-1), static_cast<Double>(1))) * 2;
  }
}

// From ROS2 tf2::Quaternion::slerp()
Orientation slerp(const Orientation & a, const Orientation & b, const Real t)
{
  const auto t_ = static_cast<Double>(clamp(t, 0.0F, 1.0F));

  Double theta = angleShortestPath(a, b) / 2;
  if (theta != 0) {
    Double d = 1 / std::sin(theta);
    Double s0 = std::sin((1 - t_) * theta);
    Double s1 = std::sin(t_ * theta);
    if (dot(a, b) < 0) {
      return Orientation{
        (a.x * s0 + -b.x * s1) * d,
        (a.y * s0 + -b.y * s1) * d,
        (a.z * s0 + -b.z * s1) * d,
        (a.w * s0 + -b.w * s1) * d};
    } else {
      return Orientation{
        (a.x * s0 + b.x * s1) * d,
        (a.y * s0 + b.y * s1) * d,
        (a.z * s0 + b.z * s1) * d,
        (a.w * s0 + b.w * s1) * d};
    }
  }

  return a;
}

////////////////////////////////////////////////////////////////////////////////
Point interpolate(Point a, Point b, Real t)
{
  return interpolate(a, b, t, slerp);
}

////////////////////////////////////////////////////////////////////////////////
void sample(const Trajectory & in, Trajectory & out, std::chrono::nanoseconds period)
{
  sample(in, out, period, slerp);
}
}  // namespace motion_common
}  // namespace motion
