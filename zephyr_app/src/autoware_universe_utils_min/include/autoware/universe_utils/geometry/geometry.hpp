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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_

// Autoware
#include "autoware/universe_utils/geometry/alt_geometry.hpp"
#include "autoware/universe_utils/math/constants.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"

// Standard library
#include <exception>
#include <string>
#include <vector>
#include <cmath>

// Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

// Message types
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

namespace autoware::universe_utils
{
// createPoint: create a point
inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// createVector3: create a vector3
inline geometry_msgs::msg::Vector3 createVector3(const double x, double y, double z)
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(z);
}

// getPoint: get the location of the point
template <class T>
geometry_msgs::msg::Point getPoint(const T & p) 
{
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(p.x).y(p.y).z(p.z);  
}
template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p)  { return p; }
template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p)  { return p.position; }
template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)  { return p.pose.position; }
template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseWithCovarianceStamped & p)  { return p.pose.pose.position; }
template <>
inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::PathPoint & p) { return p.pose.position; }
template <>
inline geometry_msgs::msg::Point getPoint(const tier4_planning_msgs::msg::PathPointWithLaneId & p)  { return p.point.pose.position; }
template <>
inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::TrajectoryPoint & p) { return p.pose.position; }

// getPose: get the pose of the point
template <class T>
geometry_msgs::msg::Pose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}
template <>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::Pose & p) { return p; }
template <>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::PoseStamped & p) { return p.pose; }
template <>
inline geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::PathPoint & p) { return p.pose; }
template <>
inline geometry_msgs::msg::Pose getPose(const tier4_planning_msgs::msg::PathPointWithLaneId & p) { return p.point.pose; }
template <>
inline geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::TrajectoryPoint & p) { return p.pose; }

// setPose: set the pose of the point
template <class T>
void setPose([[maybe_unused]] const geometry_msgs::msg::Pose & pose, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}
template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Pose & p) { p = pose; }
template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::PoseStamped & p) { p.pose = pose; }
template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, autoware_planning_msgs::msg::PathPoint & p)  { p.pose = pose; }
template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, tier4_planning_msgs::msg::PathPointWithLaneId & p) { p.point.pose = pose; }
template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, autoware_planning_msgs::msg::TrajectoryPoint & p) { p.pose = pose; }

// getLongitudinalVelocity: get the longitudinal velocity of the point
template <class T>
double getLongitudinalVelocity([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getVelocity can be used.");
  throw std::logic_error("Only specializations of getVelocity can be used.");
}
template <>
inline double getLongitudinalVelocity(const autoware_planning_msgs::msg::PathPoint & p) { return p.longitudinal_velocity_mps; }
template <>
inline double getLongitudinalVelocity(const tier4_planning_msgs::msg::PathPointWithLaneId & p) { return p.point.longitudinal_velocity_mps; }
template <>
inline double getLongitudinalVelocity(const autoware_planning_msgs::msg::TrajectoryPoint & p) { return p.longitudinal_velocity_mps; }

// setLongitudinalVelocity: set the longitudinal velocity of the point
template <class T>
void setLongitudinalVelocity([[maybe_unused]] const float velocity, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getLongitudinalVelocity can be used.");
  throw std::logic_error("Only specializations of getLongitudinalVelocity can be used.");
}
template <>
inline void setLongitudinalVelocity(const float velocity, autoware_planning_msgs::msg::TrajectoryPoint & p) { p.longitudinal_velocity_mps = velocity; }
template <>
inline void setLongitudinalVelocity(const float velocity, autoware_planning_msgs::msg::PathPoint & p) { p.longitudinal_velocity_mps = velocity; }
template <>
inline void setLongitudinalVelocity(const float velocity, tier4_planning_msgs::msg::PathPointWithLaneId & p) { p.point.longitudinal_velocity_mps = velocity; }

// createTranslation: create a translation vector
geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z);

// calcDistance2d: calculate the distance between two points
template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

// setOrientation: set the orientation of the point
template <class T>
inline void setOrientation(const geometry_msgs::msg::Quaternion & orientation, T & p)
{
  auto pose = getPose(p);
  pose.orientation = orientation;
  setPose(pose, p);
}

// getRPY: get the roll, pitch, and yaw of the point
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Quaternion & quat);
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose);
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

// createQuaternion
geometry_msgs::msg::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w);

geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw);

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw);

/**
 * @brief calculate elevation angle of two points.
 * @details This function returns the elevation angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If the two points are in the same position, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi/2 <= elevation angle <= pi/2
 */
double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
double calcAzimuthAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

template <class Point1, class Point2>
Vector3 point2tfVector(const Point1 & src, const Point2 & dst)
{
  const auto src_p = getPoint(src);
  const auto dst_p = getPoint(dst);

  double dx = dst_p.x - src_p.x;
  double dy = dst_p.y - src_p.y;
  double dz = dst_p.z - src_p.z;
  return Vector3(dx, dy, dz);
}

double calcCurvature(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3);

template <class Pose1, class Pose2>
bool isDrivingForward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = getYaw(getPose(src_pose).orientation);
  const double pose_direction_yaw = calcAzimuthAngle(getPoint(src_pose), getPoint(dst_pose));
  return std::fabs(normalizeRadian(src_yaw - pose_direction_yaw)) < pi / 2.0;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z,
  const double yaw = 0.0);

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
geometry_msgs::msg::Point calcInterpolatedPoint(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = getPoint(src);
  const auto dst_point = getPoint(dst);

  Vector3 src_vec;
  src_vec.setX(src_point.x);
  src_vec.setY(src_point.y);
  src_vec.setZ(src_point.z);

  Vector3 dst_vec;
  dst_vec.setX(dst_point.x);
  dst_vec.setY(dst_point.y);
  dst_vec.setZ(dst_point.z);

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = lerp(src_vec, dst_vec, clamped_ratio);

  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();

  return point;
}

/**
 * @brief Calculate a pose by linear interpolation.
 * Note that if dist(src_pose, dst_pose)<=0.01
 * the orientation of the output pose is same as the orientation
 * of the dst_pose
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @param set_orientation_from_position_direction set position by spherical interpolation if false
 * @return interpolated point
 */
template <class Pose1, class Pose2>
geometry_msgs::msg::Pose calcInterpolatedPose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometry_msgs::msg::Pose output_pose;
  output_pose.position =
    calcInterpolatedPoint(getPoint(src_pose), getPoint(dst_pose), clamped_ratio);

  if (set_orientation_from_position_direction) {
    const double input_poses_dist = calcDistance2d(getPoint(src_pose), getPoint(dst_pose));
    const bool is_driving_forward = isDrivingForward(src_pose, dst_pose);

    // Get orientation from interpolated point and src_pose
    if ((is_driving_forward && clamped_ratio > 1.0 - (1e-6)) || input_poses_dist < 1e-3) {
      output_pose.orientation = getPose(dst_pose).orientation;
    } else if (!is_driving_forward && clamped_ratio < 1e-6) {
      output_pose.orientation = getPose(src_pose).orientation;
    } else {
      const auto & base_pose = is_driving_forward ? dst_pose : src_pose;
      const double pitch = calcElevationAngle(getPoint(output_pose), getPoint(base_pose));
      const double yaw = calcAzimuthAngle(getPoint(output_pose), getPoint(base_pose));
      output_pose.orientation = createQuaternionFromRPY(0.0, pitch, yaw);
    }
  } else {
    // Get orientation by spherical linear interpolation
    // tf2::Transform src_tf;
    // tf2::Transform dst_tf;
    // tf2::fromMsg(getPose(src_pose), src_tf);
    // tf2::fromMsg(getPose(dst_pose), dst_tf);
    // const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    // output_pose.orientation = tf2::toMsg(quaternion);

    Quaterniond qa, qb, qres;
    // initialize qa, qb;
    qres = qa.slerp(clamped_ratio, qb);
    output_pose.orientation = toMsg(qres);
  }

  return output_pose;
}

// NOTE: much faster than boost::geometry::intersects()
std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4);

/**
 * @brief Check if 2 convex polygons intersect using the GJK algorithm
 * @details much faster than boost::geometry::intersects()
 */
bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2);

/**
 * @brief Linear interpolation between two vectors
 * @param src_vec Source vector
 * @param dst_vec Destination vector
 * @param ratio Interpolation ratio (typically between 0 and 1)
 * @return Interpolated vector
 */
geometry_msgs::msg::Vector3 lerp(
  const geometry_msgs::msg::Vector3 & src_vec,
  const geometry_msgs::msg::Vector3 & dst_vec,
  const double ratio);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_
