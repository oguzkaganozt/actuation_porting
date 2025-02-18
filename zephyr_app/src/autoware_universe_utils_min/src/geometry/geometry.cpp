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

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/geometry/gjk_2d.hpp"
#include <Eigen/Geometry>
#include <string>

namespace autoware::universe_utils
{
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Quaternion & quat)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  return getRPY(pose.orientation);
}
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose)
{
  return getRPY(pose.pose);
}
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return getRPY(pose.pose.pose);
}

geometry_msgs::msg::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  // Calculate rotation matrix elements
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  // Convert to quaternion using the rotation matrix to quaternion formulas
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  // Calculate half angles
  const double half_yaw = yaw * 0.5;
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  // When roll and pitch are 0, the quaternion simplifies to:
  geometry_msgs::msg::Quaternion q;
  q.w = cy;  // cos(0/2)cos(0/2)cos(yaw/2) = cos(yaw/2)
  q.x = 0;   // sin(0/2)cos(0/2)cos(yaw/2) = 0
  q.y = 0;   // cos(0/2)sin(0/2)cos(yaw/2) = 0
  q.z = sy;  // cos(0/2)cos(0/2)sin(yaw/2) = sin(yaw/2)

  return q;
}

geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

double calcAzimuthAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Convert quaternion to Euler angles
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double calcCurvature(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3)
{
  // Calculation details are described in the following page
  // https://en.wikipedia.org/wiki/Menger_curvature
  const double denominator =
    calcDistance2d(p1, p2) * calcDistance2d(p2, p3) * calcDistance2d(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
}

std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  // calculate intersection point
  const double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if (det == 0.0) {
    return std::nullopt;
  }

  const double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  const double s = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = t * p1.x + (1.0 - t) * p2.x;
  intersect_point.y = t * p1.y + (1.0 - t) * p2.y;
  intersect_point.z = t * p1.z + (1.0 - t) * p2.z;
  return intersect_point;
}

bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  return gjk::intersects(convex_polygon1, convex_polygon2);
}

geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z,
  const double yaw)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform transform;
  transform.translation = createTranslation(x, y, z);
  transform.rotation = createQuaternionFromYaw(yaw);
  // replace tf2 with eigen
  // tf2::Transform tf_pose;
  // tf2::Transform tf_offset;
  // tf2::fromMsg(transform, tf_offset);
  // tf2::fromMsg(p, tf_pose);
  // tf2::toMsg(tf_pose * tf_offset, pose);
  Eigen::Translation3d T(x, y, z);
  Eigen::Quaterniond R(createQuaternionFromYaw(yaw));
  Eigen::Matrix4d transform_matrix = (T * R).matrix();
  Eigen::Matrix4d pose_matrix = p.position.x, p.position.y, p.position.z, 1.0);
  Eigen::Matrix4d result_matrix = transform_matrix * pose_matrix;
  pose.position.x = result_matrix(0, 3);
  pose.position.y = result_matrix(1, 3);
  pose.position.z = result_matrix(2, 3);
  pose.orientation = createQuaternionFromRPY(0, 0, yaw);
  return pose;
}

geometry_msgs::msg::Vector3 lerp(
  const geometry_msgs::msg::Vector3 & src_vec, 
  const geometry_msgs::msg::Vector3 & dst_vec, 
  const double ratio)
{
  geometry_msgs::msg::Vector3 result;
  // Linear interpolation formula: result = src + ratio * (dst - src)
  result.x = src_vec.x + ratio * (dst_vec.x - src_vec.x);
  result.y = src_vec.y + ratio * (dst_vec.y - src_vec.y);
  result.z = src_vec.z + ratio * (dst_vec.z - src_vec.z);
  return result;
}
}  // namespace autoware::universe_utils