// Copyright 2023-2024 TIER IV, Inc.
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

#include "autoware/universe_utils/geometry/alt_geometry.hpp"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace autoware::universe_utils
{
// Alternatives for Boost.Geometry ----------------------------------------------------------------
namespace alt
{
std::optional<Polygon2d> Polygon2d::create(
  const PointList2d & outer, const std::vector<PointList2d> & inners) noexcept
{
  Polygon2d poly(outer, inners);
  correct(poly);

  if (poly.outer().size() < 4) {
    return std::nullopt;
  }

  for (const auto & inner : poly.inners()) {
    if (inner.size() < 4) {
      return std::nullopt;
    }
  }

  return poly;
}

std::optional<Polygon2d> Polygon2d::create(
  PointList2d && outer, std::vector<PointList2d> && inners) noexcept
{
  Polygon2d poly(std::move(outer), std::move(inners));
  correct(poly);

  if (poly.outer().size() < 4) {
    return std::nullopt;
  }

  for (const auto & inner : poly.inners()) {
    if (inner.size() < 4) {
      return std::nullopt;
    }
  }

  return poly;
}

std::optional<Polygon2d> Polygon2d::create(
  const autoware::universe_utils::Polygon2d & polygon) noexcept
{
  PointList2d outer;
  for (const auto & point : polygon.outer()) {
    outer.push_back(Point2d(point));
  }

  std::vector<PointList2d> inners;
  for (const auto & inner : polygon.inners()) {
    PointList2d _inner;
    if (inner.empty()) {
      continue;
    }
    for (const auto & point : inner) {
      _inner.push_back(Point2d(point));
    }
    inners.push_back(_inner);
  }

  return Polygon2d::create(outer, inners);
}

autoware::universe_utils::Polygon2d Polygon2d::to_boost() const
{
  autoware::universe_utils::Polygon2d polygon;

  for (const auto & point : outer_) {
    polygon.outer().emplace_back(point.x(), point.y());
  }

  for (const auto & inner : inners_) {
    autoware::universe_utils::LinearRing2d _inner;
    for (const auto & point : inner) {
      _inner.emplace_back(point.x(), point.y());
    }
    polygon.inners().push_back(_inner);
  }

  return polygon;
}

std::optional<ConvexPolygon2d> ConvexPolygon2d::create(const PointList2d & vertices) noexcept
{
  ConvexPolygon2d poly(vertices);
  correct(poly);

  if (poly.vertices().size() < 4) {
    return std::nullopt;
  }

  if (!is_convex(poly)) {
    return std::nullopt;
  }

  return poly;
}

std::optional<ConvexPolygon2d> ConvexPolygon2d::create(PointList2d && vertices) noexcept
{
  ConvexPolygon2d poly(std::move(vertices));
  correct(poly);

  if (poly.vertices().size() < 4) {
    return std::nullopt;
  }

  if (!is_convex(poly)) {
    return std::nullopt;
  }

  return poly;
}

std::optional<ConvexPolygon2d> ConvexPolygon2d::create(
  const autoware::universe_utils::Polygon2d & polygon) noexcept
{
  PointList2d vertices;
  for (const auto & point : polygon.outer()) {
    vertices.push_back(Point2d(point));
  }

  return ConvexPolygon2d::create(vertices);
}

double dot_product(const Point2d& p1, const Point2d& p2) {
  return p1.x() * p2.x() + p1.y() * p2.y();
}

double cross_product(const Point2d& p1, const Point2d& p2) {
  return p1.x() * p2.y() - p1.y() * p2.x();
}

double distance(const Point2d& p1, const Point2d& p2) {
  return std::sqrt(squared_distance(p1, p2));
}

double squared_distance(const Point2d& p1, const Point2d& p2) {
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  return dx * dx + dy * dy;
}

double area(const ConvexPolygon2d& poly) {
  const auto& vertices = poly.vertices();
  if (vertices.size() < 3) return 0.0;
  
  double area = 0.0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % vertices.size()];
    area += cross_product(p1, p2);
  }
  return std::abs(area) * 0.5;
}

ConvexPolygon2d convex_hull(const Points2d& points) {
  if (points.size() < 3) return ConvexPolygon2d::create(points);
  
  // Graham scan algorithm
  std::vector<Point2d> hull = points;
  
  // Find point with lowest y-coordinate (and leftmost if tied)
  auto pivot = std::min_element(hull.begin(), hull.end(),
    [](const Point2d& p1, const Point2d& p2) {
      return p1.y() < p2.y() || (p1.y() == p2.y() && p1.x() < p2.x());
    });
    
  // Sort points by polar angle with pivot
  std::sort(hull.begin(), hull.end(),
    [pivot](const Point2d& p1, const Point2d& p2) {
      double cross = cross_product(
        Point2d(p1.x() - pivot->x(), p1.y() - pivot->y()),
        Point2d(p2.x() - pivot->x(), p2.y() - pivot->y())
      );
      return cross > 0;
    });
    
  // Build convex hull
  std::vector<Point2d> result;
  for (const auto& point : hull) {
    while (result.size() >= 2) {
      const auto& p1 = result[result.size() - 2];
      const auto& p2 = result[result.size() - 1];
      if (cross_product(
        Point2d(p2.x() - p1.x(), p2.y() - p1.y()),
        Point2d(point.x() - p2.x(), point.y() - p2.y())
      ) > 0) break;
      result.pop_back();
    }
    result.push_back(point);
  }
  
  return ConvexPolygon2d::create(result);
}

bool is_clockwise(const PointList2d& points) {
  if (points.size() < 3) return false;
  
  double sum = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p1 = points[i];
    const auto& p2 = points[(i + 1) % points.size()];
    sum += (p2.x() - p1.x()) * (p2.y() + p1.y());
  }
  return sum > 0;
}

void correct(Polygon2d& poly) {
  auto& outer = poly.outer();
  if (!is_clockwise(outer)) {
    std::reverse(outer.begin(), outer.end());
  }
  
  for (auto& inner : poly.inners()) {
    if (is_clockwise(inner)) {
      std::reverse(inner.begin(), inner.end());
    }
  }
}

bool covered_by(const Point2d & point, const ConvexPolygon2d & poly)
{
  constexpr double epsilon = 1e-6;

  const auto & vertices = poly.vertices();
  std::size_t winding_number = 0;

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), std::prev(vertices.end()),
    [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() < y_min_vertex->y() || point.y() > y_max_vertex->y()) {
    return false;
  }

  double cross;
  for (auto it = vertices.cbegin(); it != std::prev(vertices.cend()); ++it) {
    const auto & p1 = *it;
    const auto & p2 = *std::next(it);

    if (p1.y() <= point.y() && p2.y() >= point.y()) {  // upward edge
      cross = cross_product(p2 - p1, point - p1);
      if (cross > 0) {  // point is to the left of edge
        winding_number++;
        continue;
      }
    } else if (p1.y() >= point.y() && p2.y() <= point.y()) {  // downward edge
      cross = cross_product(p2 - p1, point - p1);
      if (cross < 0) {  // point is to the left of edge
        winding_number--;
        continue;
      }
    } else {
      continue;
    }

    if (std::abs(cross) < epsilon) {  // point is on edge
      return true;
    }
  }

  return winding_number != 0;
}

bool disjoint(const ConvexPolygon2d & poly1, const ConvexPolygon2d & poly2)
{
  if (equals(poly1, poly2)) {
    return false;
  }

  if (intersects(poly1, poly2)) {
    return false;
  }

  for (const auto & vertex : poly1.vertices()) {
    if (touches(vertex, poly2)) {
      return false;
    }
  }

  return true;
}

double distance(
  const Point2d & point, const Point2d & seg_start, const Point2d & seg_end)
{
  constexpr double epsilon = 1e-3;

  const auto seg_vec = seg_end - seg_start;
  const auto point_vec = point - seg_start;

  const double seg_vec_norm = seg_vec.norm();
  const double seg_point_dot = seg_vec.dot(point_vec);

  if (seg_vec_norm < epsilon || seg_point_dot < 0) {
    return point_vec.norm();
  } else if (seg_point_dot > std::pow(seg_vec_norm, 2)) {
    return (point - seg_end).norm();
  } else {
    return std::abs(cross_product(seg_vec, point_vec)) / seg_vec_norm;
  }
}

double distance(const Point2d & point, const ConvexPolygon2d & poly)
{
  if (covered_by(point, poly)) {
    return 0.0;
  }

  // TODO(mitukou1109): Use plane sweep method to improve performance?
  const auto & vertices = poly.vertices();
  double min_distance = std::numeric_limits<double>::max();
  for (auto it = vertices.cbegin(); it != std::prev(vertices.cend()); ++it) {
    min_distance = std::min(min_distance, distance(point, *it, *std::next(it)));
  }

  return min_distance;
}

std::optional<ConvexPolygon2d> envelope(const Polygon2d & poly)
{
  const auto [x_min_vertex, x_max_vertex] = std::minmax_element(
    poly.outer().begin(), std::prev(poly.outer().end()),
    [](const auto & a, const auto & b) { return a.x() < b.x(); });

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    poly.outer().begin(), std::prev(poly.outer().end()),
    [](const auto & a, const auto & b) { return a.y() < b.y(); });

  return ConvexPolygon2d::create(PointList2d{
    {x_min_vertex->x(), y_min_vertex->y()},
    {x_min_vertex->x(), y_max_vertex->y()},
    {x_max_vertex->x(), y_max_vertex->y()},
    {x_max_vertex->x(), y_min_vertex->y()}});
}

bool equals(const Point2d & point1, const Point2d & point2)
{
  constexpr double epsilon = 1e-3;
  return std::abs(point1.x() - point2.x()) < epsilon && std::abs(point1.y() - point2.y()) < epsilon;
}

bool equals(const Polygon2d & poly1, const Polygon2d & poly2)
{
  // Check if outer rings have same size
  if (poly1.outer().size() != poly2.outer().size()) {
    return false;
  }

  // Check if inner rings count matches
  if (poly1.inners().size() != poly2.inners().size()) {
    return false;
  }

  // Compare outer ring points
  const auto& outer1 = poly1.outer();
  const auto& outer2 = poly2.outer();
  for (size_t i = 0; i < outer1.size(); ++i) {
    if (std::abs(outer1[i].x() - outer2[i].x()) > 1e-10 ||
        std::abs(outer1[i].y() - outer2[i].y()) > 1e-10) {
      return false;
    }
  }

  // Compare inner rings
  const auto& inners1 = poly1.inners();
  const auto& inners2 = poly2.inners();
  for (size_t i = 0; i < inners1.size(); ++i) {
    if (inners1[i].size() != inners2[i].size()) {
      return false;
    }
    for (size_t j = 0; j < inners1[i].size(); ++j) {
      if (std::abs(inners1[i][j].x() - inners2[i][j].x()) > 1e-10 ||
          std::abs(inners1[i][j].y() - inners2[i][j].y()) > 1e-10) {
        return false;
      }
    }
  }

  return true;
}

PointList2d::const_iterator find_farthest(
  const PointList2d & points, const Point2d & seg_start, const Point2d & seg_end)
{
  const auto seg_vec = seg_end - seg_start;
  return std::max_element(points.begin(), points.end(), [&](const auto & a, const auto & b) {
    return std::abs(cross_product(seg_vec, a - seg_start)) < std::abs(cross_product(seg_vec, b - seg_start));
  });
}

bool intersects(
  const Point2d & seg1_start, const Point2d & seg1_end, const Point2d & seg2_start,
  const Point2d & seg2_end)
{
  constexpr double epsilon = 1e-6;

  const auto v1 = seg1_end - seg1_start;
  const auto v2 = seg2_end - seg2_start;

  const auto det = cross_product(v1, v2);
  if (std::abs(det) < epsilon) {
    return false;
  }

  const auto v12 = seg2_end - seg1_end;
  const double t = cross_product(v2, v12) / det;
  const double s = cross_product(v1, v12) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return false;
  }

  return true;
}

bool intersects(const ConvexPolygon2d & poly1, const ConvexPolygon2d & poly2)
{
  if (equals(poly1, poly2)) {
    return true;
  }

  // GJK algorithm

  auto find_support_vector = [](
                               const ConvexPolygon2d & poly1,
                               const ConvexPolygon2d & poly2,
                               const Vector2d & direction) {
    auto find_farthest_vertex =
      [](const ConvexPolygon2d & poly, const Vector2d & direction) {
        return std::max_element(
          poly.vertices().begin(), std::prev(poly.vertices().end()),
          [&](const auto & a, const auto & b) { return direction.dot(a) <= direction.dot(b); });
      };
    return *find_farthest_vertex(poly1, direction) - *find_farthest_vertex(poly2, -direction);
  };

  Vector2d direction = {1.0, 0.0};
  auto a = find_support_vector(poly1, poly2, direction);
  direction = -a;
  auto b = find_support_vector(poly1, poly2, direction);
  if (b.dot(direction) <= 0.0) {
    return false;
  }

  direction = (b - a).vector_triple(-a, b - a);
  while (true) {
    auto c = find_support_vector(poly1, poly2, direction);
    if (c.dot(direction) <= 0.0) {
      return false;
    }

    auto n_ca = (b - c).vector_triple(a - c, a - c);
    if (n_ca.dot(-c) > 0.0) {
      b = c;
      direction = n_ca;
    } else {
      auto n_cb = (a - c).vector_triple(b - c, b - c);
      if (n_cb.dot(-c) > 0.0) {
        a = c;
        direction = n_cb;
      } else {
        break;
      }
    }
  }

  return true;
}

bool is_above(
  const Point2d & point, const Point2d & seg_start, const Point2d & seg_end)
{
  return cross_product(seg_end - seg_start, point - seg_start) > 0;
}

bool is_convex(const Polygon2d & poly)
{
  constexpr double epsilon = 1e-6;

  if (!poly.inners().empty()) {
    return false;
  }

  const auto & outer = poly.outer();

  for (auto it = std::next(outer.cbegin()); it != std::prev(outer.cend()); ++it) {
    const auto & p1 = *--it;
    const auto & p2 = *it;
    const auto & p3 = *++it;

    if (cross_product(p2 - p1, p3 - p2) > epsilon) {
      return false;
    }
  }

  return true;
}

PointList2d simplify(const PointList2d & line, const double max_distance)
{
  if (line.size() < 3) {
    return line;
  }

  Points2d pending(std::next(line.begin()), std::prev(line.end()));
  PointList2d simplified;

  // Douglas-Peucker algorithm

  auto douglas_peucker = [&max_distance, &pending, &simplified](
                           auto self, const Point2d & seg_start,
                           const Point2d & seg_end) {
    if (pending.empty()) {
      return;
    }

    const auto farthest_itr = find_farthest(pending, seg_start, seg_end);
    const auto farthest = *farthest_itr;
    pending.erase(farthest_itr);

    if (distance(farthest, seg_start, seg_end) <= max_distance) {
      return;
    }

    self(self, seg_start, farthest);
    simplified.push_back(farthest);
    self(self, farthest, seg_end);
  };

  simplified.push_back(line.front());
  douglas_peucker(douglas_peucker, line.front(), line.back());
  simplified.push_back(line.back());

  return simplified;
}

bool touches(
  const Point2d & point, const Point2d & seg_start, const Point2d & seg_end)
{
  constexpr double epsilon = 1e-6;

  // if the cross product of the vectors from the start point and the end point to the point is 0
  // and the vectors opposite each other, the point is on the segment
  const auto start_vec = point - seg_start;
  const auto end_vec = point - seg_end;
  return std::abs(cross_product(start_vec, end_vec)) < epsilon && dot_product(start_vec, end_vec) <= 0;
}

bool touches(const Point2d & point, const ConvexPolygon2d & poly)
{
  const auto & vertices = poly.vertices();

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), std::prev(vertices.end()),
    [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() < y_min_vertex->y() || point.y() > y_max_vertex->y()) {
    return false;
  }

  for (auto it = vertices.cbegin(); it != std::prev(vertices.cend()); ++it) {
    // check if the point is on each edge of the polygon
    if (touches(point, *it, *std::next(it))) {
      return true;
    }
  }

  return false;
}

bool within(const Point2d & point, const ConvexPolygon2d & poly)
{
  constexpr double epsilon = 1e-6;

  const auto & vertices = poly.vertices();
  int64_t winding_number = 0;

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), std::prev(vertices.end()),
    [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() <= y_min_vertex->y() || point.y() >= y_max_vertex->y()) {
    return false;
  }

  double cross;
  for (auto it = vertices.cbegin(); it != std::prev(vertices.cend()); ++it) {
    const auto & p1 = *it;
    const auto & p2 = *std::next(it);

    if (p1.y() < point.y() && p2.y() > point.y()) {  // upward edge
      cross = cross_product(p2 - p1, point - p1);
      if (cross > 0) {  // point is to the left of edge
        winding_number++;
        continue;
      }
    } else if (p1.y() > point.y() && p2.y() < point.y()) {  // downward edge
      cross = cross_product(p2 - p1, point - p1);
      if (cross < 0) {  // point is to the left of edge
        winding_number--;
        continue;
      }
    } else {
      continue;
    }

    if (std::abs(cross) < epsilon) {  // point is on edge
      return false;
    }
  }

  return winding_number != 0;
}

bool within(
  const ConvexPolygon2d & poly_contained, const ConvexPolygon2d & poly_containing)
{
  if (equals(poly_contained, poly_containing)) {
    return true;
  }

  // check if all points of poly_contained are within poly_containing
  for (const auto & vertex : poly_contained.vertices()) {
    if (!within(vertex, poly_containing)) {
      return false;
    }
  }

  return true;
}

}  // namespace autoware::universe_utils
