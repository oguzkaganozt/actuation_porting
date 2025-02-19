// Copyright 2023 TIER IV, Inc.
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

#include "autoware/universe_utils/ros/marker_helper.hpp"

#include <string>

namespace autoware::universe_utils
{
visualizationMsgsMarker createDefaultMarker(
  const std::string & frame_id, const rclcpp::Time & now, const std::string & ns, const int32_t id,
  const int32_t type, const geometryMsgsVector3 & scale,
  const stdMsgsColorRGBA & color)
{
  visualizationMsgsMarker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = now;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualizationMsgsMarker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = scale;
  marker.color = color;
  marker.frame_locked = true;

  return marker;
}

visualizationMsgsMarker createDeletedDefaultMarker(
  const rclcpp::Time & now, const std::string & ns, const int32_t id)
{
  visualizationMsgsMarker marker;

  marker.header.stamp = now;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualizationMsgsMarker::DELETE;

  return marker;
}

void appendMarkerArray(
  const visualizationMsgsMarkerArray & additional_marker_array,
  visualizationMsgsMarkerArray * marker_array,
  const std::optional<rclcpp::Time> & current_time)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);

    if (current_time) {
      marker_array->markers.back().header.stamp = current_time.value();
    }
  }
}

}  // namespace autoware::universe_utils
