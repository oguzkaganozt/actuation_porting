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

// Messages
#include "autoware/universe_utils/messages.h"

namespace autoware::universe_utils
{
/**
 * @brief Get the location of a point
 * @param p The point
 * @return The location
 */
template <class T>
geometryMsgsPoint getPoint(const T & p) 
{
  geometryMsgsPoint point;
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  return point;
}
template <>
inline geometryMsgsPoint getPoint(const geometryMsgsPoint & p)  { return p; }
template <>
inline geometryMsgsPoint getPoint(const geometryMsgsPose & p)  { return p.position; }
template <>
inline geometryMsgsPoint getPoint(const geometryMsgsPoseStamped & p)  { return p.pose.position; }
template <>
inline geometryMsgsPoint getPoint(const geometryMsgsPoseWithCovarianceStamped & p)  { return p.pose.pose.position; }
template <>
inline geometryMsgsPoint getPoint(const awfPlanningMsgsPathPoint & p) { return p.pose.position; }
template <>
inline geometryMsgsPoint getPoint(const tier4PlanningMsgsPathPointWithLaneId & p)  { return p.point.pose.position; }
template <>
inline geometryMsgsPoint getPoint(const awfPlanningMsgsTrajectoryPoint & p) { return p.pose.position; }

/**
 * @brief Get the pose of a point
 * @param p The point
 * @return The pose
 */
template <class T>
geometryMsgsPose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}
template <>
inline geometryMsgsPose getPose(const geometryMsgsPose & p) { return p; }
template <>
inline geometryMsgsPose getPose(const geometryMsgsPoseStamped & p) { return p.pose; }
template <>
inline geometryMsgsPose getPose(const awfPlanningMsgsPathPoint & p) { return p.pose; }
template <>
inline geometryMsgsPose getPose(const tier4PlanningMsgsPathPointWithLaneId & p) { return p.point.pose; }
template <>
inline geometryMsgsPose getPose(const awfPlanningMsgsTrajectoryPoint & p) { return p.pose; }

/**
 * @brief Set the pose of a point
 * @param pose The pose
 * @param p The point
 */
template <class T>
void setPose([[maybe_unused]] const geometryMsgsPose & pose, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}
template <>
inline void setPose(const geometryMsgsPose & pose, geometryMsgsPose & p) { p = pose; }
template <>
inline void setPose(const geometryMsgsPose & pose, geometryMsgsPoseStamped & p) { p.pose = pose; }
template <>
inline void setPose(const geometryMsgsPose & pose, awfPlanningMsgsPathPoint & p)  { p.pose = pose; }
template <>
inline void setPose(const geometryMsgsPose & pose, tier4PlanningMsgsPathPointWithLaneId & p) { p.point.pose = pose; }
template <>
inline void setPose(const geometryMsgsPose & pose, awfPlanningMsgsTrajectoryPoint & p) { p.pose = pose; }

/**
 * @brief Get the longitudinal velocity of a point
 * @param p The point
 * @return The longitudinal velocity
 */
template <class T>
double getLongitudinalVelocity([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getVelocity can be used.");
  throw std::logic_error("Only specializations of getVelocity can be used.");
}
template <>
inline double getLongitudinalVelocity(const awfPlanningMsgsPathPoint & p) { return p.longitudinal_velocity_mps; }
template <>
inline double getLongitudinalVelocity(const tier4PlanningMsgsPathPointWithLaneId & p) { return p.point.longitudinal_velocity_mps; }
template <>
inline double getLongitudinalVelocity(const awfPlanningMsgsTrajectoryPoint & p) { return p.longitudinal_velocity_mps; }

/**
 * @brief Set the longitudinal velocity of a point
 * @param velocity The longitudinal velocity
 * @param p The point
 */
template <class T>
void setLongitudinalVelocity([[maybe_unused]] const float velocity, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getLongitudinalVelocity can be used.");
  throw std::logic_error("Only specializations of getLongitudinalVelocity can be used.");
}
template <>
inline void setLongitudinalVelocity(const float velocity, awfPlanningMsgsTrajectoryPoint & p) { p.longitudinal_velocity_mps = velocity; }
template <>
inline void setLongitudinalVelocity(const float velocity, awfPlanningMsgsPathPoint & p) { p.longitudinal_velocity_mps = velocity; }
template <>
inline void setLongitudinalVelocity(const float velocity, tier4PlanningMsgsPathPointWithLaneId & p) { p.point.longitudinal_velocity_mps = velocity; }

/**
 * @brief Create a translation vector
 * @param x The x coordinate
 * @param y The y coordinate
 * @param z The z coordinate
 * @return The translation vector
 */
geometryMsgsVector3 createTranslation(const double x, const double y, const double z);

/**
 * @brief Calculate the distance between two points
 * @param point1 The first point
 * @param point2 The second point
 * @return The distance between the two points
 */
template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/**
 * @brief Set the orientation of a point
 * @param orientation The orientation
 * @param p The point
 */
template <class T>
inline void setOrientation(const geometryMsgsQuaternion & orientation, T & p)
{
  auto pose = getPose(p);
  pose.orientation = orientation;
  setPose(pose, p);
}

/**
 * @brief Get the roll, pitch, and yaw of a quaternion
 * @param quat The quaternion
 * @return The roll, pitch, and yaw
 */
geometryMsgsVector3 getRPY(const geometryMsgsQuaternion & quat);
geometryMsgsVector3 getRPY(const geometryMsgsPose & pose);
geometryMsgsVector3 getRPY(const geometryMsgsPoseStamped & pose);
geometryMsgsVector3 getRPY(const geometryMsgsPoseWithCovarianceStamped & pose);

/**
 * @brief Create a quaternion from a x, y, z, and w value
 * @param x The x value
 * @param y The y value
 * @param z The z value
 * @param w The w value
 * @return The quaternion
 */
geometryMsgsQuaternion createQuaternion(
  const double x, const double y, const double z, const double w);

/**
 * @brief Get the yaw of a quaternion
 * @param q The quaternion
 * @return The yaw
 */
double getYaw(const geometryMsgsQuaternion & q);

/**
 * @brief Create a quaternion from a roll, pitch, and yaw angle
 * @param roll The roll angle
 * @param pitch The pitch angle
 * @param yaw The yaw angle
 * @return The quaternion
 */
geometryMsgsQuaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw);

/**
 * @brief Create a quaternion from a yaw angle
 * @param yaw The yaw angle
 * @return The quaternion
 */
geometryMsgsQuaternion createQuaternionFromYaw(const double yaw);

/**
 * @brief Linear interpolation between two vectors
 * @param src_vec Source vector
 * @param dst_vec Destination vector
 * @param ratio Interpolation ratio (typically between 0 and 1)
 * @return Interpolated vector
 */
geometryMsgsVector3 lerp(
  const geometryMsgsVector3 & src_vec,
  const geometryMsgsVector3 & dst_vec,
  const double ratio);

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
  const geometryMsgsPoint & p_from, const geometryMsgsPoint & p_to);

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
  const geometryMsgsPoint & p_from, const geometryMsgsPoint & p_to);

/**
 * @brief Calculate the curvature of a path
 * @param p1 The first point
 * @param p2 The second point
 * @param p3 The third point
 * @return The curvature
 */
double calcCurvature(
  const geometryMsgsPoint & p1, const geometryMsgsPoint & p2,
  const geometryMsgsPoint & p3);

/**
 * @brief Check if a vehicle is driving forward
 * @param src_pose The source pose
 * @param dst_pose The destination pose
 * @return true if the vehicle is driving forward, false otherwise
 */
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
 * @param p The input pose
 * @param x The x offset
 * @param y The y offset
 * @param z The z offset
 * @param yaw The yaw offset
 * @return The offset pose
 */
geometryMsgsPose calcOffsetPose(
  const geometryMsgsPose & p, const double x, const double y, const double z,
  const double yaw = 0.0);

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
geometryMsgsPoint calcInterpolatedPoint(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = getPoint(src);
  const auto dst_point = getPoint(dst);

  geometryMsgsVector3 src_vec;
  src_vec.x = src_point.x;
  src_vec.y = src_point.y;
  src_vec.z = src_point.z;

  geometryMsgsVector3 dst_vec;
  dst_vec.x = dst_point.x;
  dst_vec.y = dst_point.y;
  dst_vec.z = dst_point.z;

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = lerp(src_vec, dst_vec, clamped_ratio);

  geometryMsgsPoint point;
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
geometryMsgsPose calcInterpolatedPose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometryMsgsPose output_pose;
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
    // TODO: replace with eigen
    // Get orientation by spherical linear interpolation
    // tf2::Transform src_tf;
    // tf2::Transform dst_tf;
    // tf2::fromMsg(getPose(src_pose), src_tf);
    // tf2::fromMsg(getPose(dst_pose), dst_tf);
    // const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    // output_pose.orientation = tf2::toMsg(quaternion);

    // Quaterniond qa, qb, qres;
    // // initialize qa, qb;
    // qres = qa.slerp(clamped_ratio, qb);
    // output_pose.orientation = toMsg(qres);
  }

  return output_pose;
}

/**
 * @brief Calculate the intersection point of two lines
 * @param p1 The first point of the first line
 * @param p2 The second point of the first line
 * @param p3 The first point of the second line
 * @param p4 The second point of the second line
 * @return The intersection point
 */
std::optional<geometryMsgsPoint> intersect(
  const geometryMsgsPoint & p1, const geometryMsgsPoint & p2,
  const geometryMsgsPoint & p3, const geometryMsgsPoint & p4);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_