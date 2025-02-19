// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_

#include <optional>
#include <string>

// Message types
#include "autoware/universe_utils/messages.h"

namespace autoware::universe_utils
{
inline geometryMsgsPoint createMarkerPosition(double x, double y, double z)
{
  geometryMsgsPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline geometryMsgsQuaternion createMarkerOrientation(
  double x, double y, double z, double w)
{
  geometryMsgsQuaternion quaternion;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;
  return quaternion;
}

inline geometryMsgsVector3 createMarkerScale(double x, double y, double z)
{
  geometryMsgsVector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

inline stdMsgsColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  stdMsgsColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualizationMsgsMarker createDefaultMarker(
  const std::string & frame_id, const rclcpp::Time & now, const std::string & ns, const int32_t id,
  const int32_t type, const geometryMsgsVector3 & scale,
  const stdMsgsColorRGBA & color);

visualizationMsgsMarker createDeletedDefaultMarker(
  const rclcpp::Time & now, const std::string & ns, const int32_t id);

void appendMarkerArray(
  const visualizationMsgsMarkerArray & additional_marker_array,
  visualizationMsgsMarkerArray * marker_array,
  const std::optional<rclcpp::Time> & current_time = {});

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_
