// Copyright 2022 TIER IV, Inc.
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

#include "autoware/universe_utils/geometry/polygon_utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/geometry/alt_geometry.hpp"

namespace
{
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

void appendPointToPolygon(Polygon2d & polygon, const Point2d & point)
{
  bg::append(polygon.outer(), point);
}

/*
 * NOTE: Area is negative when footprint.points is clock wise.
 *       Area is positive when footprint.points is anti clock wise.
 */
double getPolygonArea(const geometry_msgs::msg::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}
}  // namespace

namespace autoware::universe_utils
{

}  // namespace autoware::universe_utils
