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

geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
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

double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

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
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

Point3d transformPoint(const Point3d & point, const geometry_msgs::msg::Transform & transform)
{
  const auto & translation = transform.translation;
  const auto & rotation = transform.rotation;

  const Eigen::Translation3d T(translation.x, translation.y, translation.z);
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);

  const Eigen::Vector3d transformed(T * R * point);

  return Point3d{transformed.x(), transformed.y(), transformed.z()};
}

Point2d transformPoint(const Point2d & point, const geometry_msgs::msg::Transform & transform)
{
  Point3d point_3d{point.x(), point.y(), 0};
  const auto transformed = transformPoint(point_3d, transform);
  return Point2d{transformed.x(), transformed.y()};
}

Eigen::Vector3d transformPoint(const Eigen::Vector3d & point, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;

  Point3d p = transformPoint(Point3d(point.x(), point.y(), point.z()), transform);
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Vector3d vec = Eigen::Vector3d(point.x, point.y, point.z);
  auto transformed_vec = transformPoint(vec, pose);

  geometry_msgs::msg::Point transformed_point;
  transformed_point.x = transformed_vec.x();
  transformed_point.y = transformed_vec.y();
  transformed_point.z = transformed_vec.z();
  return transformed_point;
}

geometry_msgs::msg::Point32 transformPoint(
  const geometry_msgs::msg::Point32 & point32, const geometry_msgs::msg::Pose & pose)
{
  const auto point =
    geometry_msgs::build<geometry_msgs::msg::Point>().x(point32.x).y(point32.y).z(point32.z);
  const auto transformed_point = autoware::universe_utils::transformPoint(point, pose);
  return geometry_msgs::build<geometry_msgs::msg::Point32>()
    .x(transformed_point.x)
    .y(transformed_point.y)
    .z(transformed_point.z);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & pose_transform)
{
  // Replace tf2 transform with direct quaternion multiplication
  geometry_msgs::msg::Pose transformed_pose;
  
  // Position transformation
  transformed_pose.position.x = pose_transform.position.x + pose.position.x;
  transformed_pose.position.y = pose_transform.position.y + pose.position.y;
  transformed_pose.position.z = pose_transform.position.z + pose.position.z;
  
  // Orientation transformation using quaternion multiplication
  // w1*w2 - x1*x2 - y1*y2 - z1*z2
  transformed_pose.orientation.w = 
    pose_transform.orientation.w * pose.orientation.w -
    pose_transform.orientation.x * pose.orientation.x -
    pose_transform.orientation.y * pose.orientation.y -
    pose_transform.orientation.z * pose.orientation.z;
    
  // w1*x2 + x1*w2 + y1*z2 - z1*y2
  transformed_pose.orientation.x =
    pose_transform.orientation.w * pose.orientation.x +
    pose_transform.orientation.x * pose.orientation.w +
    pose_transform.orientation.y * pose.orientation.z -
    pose_transform.orientation.z * pose.orientation.y;
    
  // w1*y2 - x1*z2 + y1*w2 + z1*x2
  transformed_pose.orientation.y =
    pose_transform.orientation.w * pose.orientation.y -
    pose_transform.orientation.x * pose.orientation.z +
    pose_transform.orientation.y * pose.orientation.w +
    pose_transform.orientation.z * pose.orientation.x;
    
  // w1*z2 + x1*y2 - y1*x2 + z1*w2
  transformed_pose.orientation.z =
    pose_transform.orientation.w * pose.orientation.z +
    pose_transform.orientation.x * pose.orientation.y -
    pose_transform.orientation.y * pose.orientation.x +
    pose_transform.orientation.z * pose.orientation.w;

  return transformed_pose;
}

// Replace tf2::getYaw with direct calculation
double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Convert quaternion to Euler angles
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform = transform;

  return transformPose(pose, transform_stamped);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & pose_transform)
{
  tf2::Transform transform;
  tf2::convert(pose_transform, transform);

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.transform = tf2::toMsg(transform);

  return transformPose(pose, transform_msg);
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

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z,
  const double yaw)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform transform;
  transform.translation = createTranslation(x, y, z);
  transform.rotation = createQuaternionFromYaw(yaw);
  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(p, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, pose);
  return pose;
}

// NOTE: much faster than boost::geometry::intersects()
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
