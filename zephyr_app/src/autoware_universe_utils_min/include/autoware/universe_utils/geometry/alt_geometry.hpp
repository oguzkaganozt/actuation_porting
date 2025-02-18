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

using Points2d = std::vector<Point2d>;
using PointList2d = std::vector<Point2d>;

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

// Geometric operations
double area(const ConvexPolygon2d& poly);
ConvexPolygon2d convex_hull(const Points2d& points);
void correct(Polygon2d& poly);
bool covered_by(const Point2d& point, const ConvexPolygon2d& poly);
bool equals(const Polygon2d& poly1, const Polygon2d& poly2);
bool is_convex(const Polygon2d& poly);
PointList2d simplify(const PointList2d& line, const double max_distance);
bool touches(const Point2d& point, const Point2d& seg_start, const Point2d& seg_end);
bool touches(const Point2d& point, const ConvexPolygon2d& poly);
bool within(const Point2d& point, const ConvexPolygon2d& poly);
bool within(const ConvexPolygon2d& poly_contained, const ConvexPolygon2d& poly_containing);

// Helper functions
double dot_product(const Point2d& p1, const Point2d& p2);
double cross_product(const Point2d& p1, const Point2d& p2);
double distance(const Point2d& p1, const Point2d& p2);
double squared_distance(const Point2d& p1, const Point2d& p2);

// Polygon operations
Polygon2d rotatePolygon(const Polygon2d& polygon, double angle);
Polygon2d translatePolygon(const Polygon2d& polygon, double x, double y);

// Conversion operations
Polygon2d msgToPolygon2d(const geometry_msgs::msg::Polygon& polygon_msg);
geometry_msgs::msg::Polygon polygon2dToMsg(const Polygon2d& polygon);

// Utility operations
bool isValid(const Polygon2d& polygon);
void makeClockwise(Polygon2d& polygon);
void makeCounterClockwise(Polygon2d& polygon);
} // namespace autoware::universe_utils::alt

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__ALT_GEOMETRY_HPP_

/*USAGE EXAMPLE:
using namespace autoware::universe_utils::alt;

// Create a triangle
LinearRing2d ring;
ring.push_back(Point2d(0, 0));
ring.push_back(Point2d(1, 0));
ring.push_back(Point2d(0, 1));
ring.close();  // Adds first point again to close the ring

// Check if it's closed
assert(ring.is_closed());
assert(ring.size() == 4);  // 3 vertices + 1 closing point
*/