// Copyright 2020-2024 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__ALT_GEOMETRY_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__ALT_GEOMETRY_HPP_

#include <Eigen/Core>
#include <vector>
#include <optional>
#include <cmath>

namespace autoware::universe_utils::alt {

/**
 * @brief A 2D point class that represents a point in 2D space
 */
class Point2d {
public:
  Point2d() : x_(0.0), y_(0.0) {}
  Point2d(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double& x() { return x_; }
  double& y() { return y_; }

private:
  double x_;
  double y_;
};

// alias types
using Points2d = std::vector<Point2d>;
using PointList2d = std::vector<Point2d>;

/**
 * @brief A polygon class that represents a polygon
 */
class Polygon2d {
public:
  explicit Polygon2d(const PointList2d& outer = {}, const std::vector<PointList2d>& inners = {})
    : outer_(outer), inners_(inners) {}

  const PointList2d& outer() const { return outer_; }
  PointList2d& outer() { return outer_; }
  const std::vector<PointList2d>& inners() const { return inners_; }
  std::vector<PointList2d>& inners() { return inners_; }

private:
  PointList2d outer_;
  std::vector<PointList2d> inners_;
};

/**
 * @brief A convex polygon class that represents a convex polygon
 */
class ConvexPolygon2d : public Polygon2d {
public:
  static ConvexPolygon2d create(const PointList2d& vertices) noexcept;
  static ConvexPolygon2d create(PointList2d&& vertices) noexcept;
  static ConvexPolygon2d create(const Polygon2d& polygon) noexcept;

  const PointList2d& vertices() const noexcept { return outer(); }
  PointList2d& vertices() noexcept { return outer(); }

private:
  explicit ConvexPolygon2d(const PointList2d& vertices) : Polygon2d(vertices, {}) {}
  explicit ConvexPolygon2d(PointList2d&& vertices) : Polygon2d(std::move(vertices), {}) {}
};

/**
 * @brief A simple 2D linear ring class that represents a closed sequence of points
 */
class LinearRing2d
{
public:
  using Points = std::vector<Point2d>;
  using iterator = Points::iterator;
  using const_iterator = Points::const_iterator;

  LinearRing2d() = default;
  explicit LinearRing2d(const Points & points) : points_(points) {}

  // Access points
  Points & points() { return points_; }
  const Points & points() const { return points_; }

  // Size operations
  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }

  // Element access
  Point2d & operator[](size_t i) { return points_[i]; }
  const Point2d & operator[](size_t i) const { return points_[i]; }
  Point2d & at(size_t i) { return points_.at(i); }
  const Point2d & at(size_t i) const { return points_.at(i); }

  // Iterators
  iterator begin() { return points_.begin(); }
  const_iterator begin() const { return points_.begin(); }
  iterator end() { return points_.end(); }
  const_iterator end() const { return points_.end(); }

  // Modifiers
  void push_back(const Point2d & point) { points_.push_back(point); }
  void clear() { points_.clear(); }
  void resize(size_t n) { points_.resize(n); }
  void reserve(size_t n) { points_.reserve(n); }

  // Check if ring is closed (first point equals last point)
  bool is_closed() const
  {
    return !empty() && points_.front() == points_.back();
  }

  // Close the ring by adding first point at the end if needed
  void close()
  {
    if (!is_closed() && !empty()) {
      points_.push_back(points_.front());
    }
  }

private:
  Points points_;
};

/**
 * @brief Calculate the area of a convex polygon
 * @param poly The convex polygon to calculate the area of
 * @return The area of the polygon
 */
double area(const ConvexPolygon2d& poly);

/**
 * @brief Calculate the convex hull of a set of points
 * @param points The points to calculate the convex hull of
 * @return The convex hull of the points
 */
ConvexPolygon2d convex_hull(const Points2d& points);

/**
 * @brief Correct a polygon
 * @param poly The polygon to correct
 */
void correct(Polygon2d& poly);

/**
 * @brief Check if a point is covered by a convex polygon
 * @param point The point to check
 * @param poly The convex polygon to check against
 */
bool covered_by(const Point2d& point, const ConvexPolygon2d& poly);

/**
 * @brief Check if two polygons are equal
 * @param poly1 The first polygon
 * @param poly2 The second polygon
 * @return true if the polygons are equal, false otherwise
 */
bool equals(const Polygon2d& poly1, const Polygon2d& poly2);

/**
 * @brief Check if a polygon is convex
 * @param poly The polygon to check
 * @return true if the polygon is convex, false otherwise
 */
bool is_convex(const Polygon2d& poly);

/**
 * @brief Simplify a polygon
 * @param line The polygon to simplify
 * @param max_distance The maximum distance between points
 * @return The simplified polygon
 */
PointList2d simplify(const PointList2d& line, const double max_distance);

/**
 * @brief Check if a point touches a segment
 * @param point The point to check
 * @param seg_start The start point of the segment
 * @param seg_end The end point of the segment
 * @return true if the point touches the segment, false otherwise
 */
bool touches(const Point2d& point, const Point2d& seg_start, const Point2d& seg_end);
bool touches(const Point2d& point, const ConvexPolygon2d& poly);

/**
 * @brief Check if a point is within a convex polygon
 * @param point The point to check
 * @param poly The convex polygon to check against
 * @return true if the point is within the polygon, false otherwise
 */
bool within(const Point2d& point, const ConvexPolygon2d& poly);
bool within(const ConvexPolygon2d& poly_contained, const ConvexPolygon2d& poly_containing);

// Helper functions
double dot_product(const Point2d& p1, const Point2d& p2);
double cross_product(const Point2d& p1, const Point2d& p2);
Point2d cross_product(const Point2d & p1, const Point2d & p2, const Point2d & p3)
double distance(const Point2d& p1, const Point2d& p2);
double squared_distance(const Point2d& p1, const Point2d& p2);
} // namespace autoware::universe_utils::alt

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__ALT_GEOMETRY_HPP_