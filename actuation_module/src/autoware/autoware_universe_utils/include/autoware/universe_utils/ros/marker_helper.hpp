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

// Msgs
#include "Point.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Marker.h"
#include "MarkerArray.h"
#include "ColorRGBA.h"
using PointMsg = geometry_msgs_msg_Point;
using QuaternionMsg = geometry_msgs_msg_Quaternion;
using Vector3Msg = geometry_msgs_msg_Vector3;
using MarkerMsg = visualization_msgs_msg_Marker;
using MarkerArrayMsg = visualization_msgs_msg_MarkerArray;
using ColorRGBA = std_msgs_msg_ColorRGBA;

namespace autoware::universe_utils
{
inline PointMsg createMarkerPosition(double x, double y, double z)
{
  PointMsg point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline QuaternionMsg createMarkerOrientation(
  double x, double y, double z, double w)
{
  QuaternionMsg quaternion;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;
  return quaternion;
}

inline Vector3Msg createMarkerScale(double x, double y, double z)
{
  Vector3Msg scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

inline ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

MarkerMsg createDefaultMarker(
  const std::string & frame_id, const double now, const std::string & ns, const int32_t id,
  const int32_t type, const Vector3Msg & scale,
  const ColorRGBA & color);

MarkerMsg createDeletedDefaultMarker(
  const double now, const std::string & ns, const int32_t id);

void appendMarkerArray(
  const MarkerArrayMsg & additional_marker_array,
  MarkerArrayMsg * marker_array,
  const std::optional<double> & current_time = {});

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_
