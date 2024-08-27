// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_geometry/src/spatial_hash.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <geometry/spatial_hash.hpp>
#include <geometry_msgs/msg/point32.hpp>
//lint -e537 NOLINT repeated include file due to cpplint rule
#include <algorithm>
//lint -e537 NOLINT repeated include file due to cpplint rule
#include <limits>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace spatial_hash
{
////////////////////////////////////////////////////////////////////////////////
Config2d::Config2d(
  const float32_t min_x,
  const float32_t max_x,
  const float32_t min_y,
  const float32_t max_y,
  const float32_t radius,
  const Index capacity)
: Config(min_x, max_x, min_y, max_y, {}, std::numeric_limits<float32_t>::min(),
    radius, capacity)
{
}
////////////////////////////////////////////////////////////////////////////////
Index Config2d::bin_(const float32_t x, const float32_t y, const float32_t z) const
{
  (void)z;
  return bin_impl(x, y);
}
////////////////////////////////////////////////////////////////////////////////
bool Config2d::valid(
  const details::Index3 & ref,
  const details::Index3 & query,
  const float ref_distance2) const
{
  const float dx = idx_distance(ref.x, query.x);
  const float dy = idx_distance(ref.y, query.y);
  // minor algebraic manipulation happening below
  return (((dx * dx) + (dy * dy)) * side_length2()) <= ref_distance2;
}
////////////////////////////////////////////////////////////////////////////////
details::Index3 Config2d::index3_(const float32_t x, const float32_t y, const float32_t z) const
{
  (void)z;
  return {x_index(x), y_index(y), Index{}};  // zero initialization
}
////////////////////////////////////////////////////////////////////////////////
Index Config2d::index_(const details::Index3 & idx) const
{
  return bin_impl(idx.x, idx.y);
}
////////////////////////////////////////////////////////////////////////////////
Config3d::Config3d(
  const float32_t min_x,
  const float32_t max_x,
  const float32_t min_y,
  const float32_t max_y,
  const float32_t min_z,
  const float32_t max_z,
  const float32_t radius,
  const Index capacity)
: Config(min_x, max_x, min_y, max_y, min_z, max_z, radius, capacity)
{
}
////////////////////////////////////////////////////////////////////////////////
Index Config3d::bin_(const float32_t x, const float32_t y, const float32_t z) const
{
  return bin_impl(x, y, z);
}
////////////////////////////////////////////////////////////////////////////////
bool Config3d::valid(
  const details::Index3 & ref,
  const details::Index3 & query,
  const float ref_distance2) const
{
  const float dx = idx_distance(ref.x, query.x);
  const float dy = idx_distance(ref.y, query.y);
  const float dz = idx_distance(ref.z, query.z);
  // minor algebraic manipulation happening below
  return (((dx * dx) + (dy * dy) + (dz * dz)) * side_length2()) <= ref_distance2;
}
////////////////////////////////////////////////////////////////////////////////
details::Index3 Config3d::index3_(const float32_t x, const float32_t y, const float32_t z) const
{
  return {x_index(x), y_index(y), z_index(z)};  // zero initialization
}
////////////////////////////////////////////////////////////////////////////////
Index Config3d::index_(const details::Index3 & idx) const
{
  return bin_impl(idx.x, idx.y, idx.z);
}
////////////////////////////////////////////////////////////////////////////////
template class SpatialHash<geometry_msgs::msg::Point32, Config2d>;
template class SpatialHash<geometry_msgs::msg::Point32, Config3d>;
}  // namespace spatial_hash
}  // namespace geometry
}  // namespace common
}  // namespace autoware
