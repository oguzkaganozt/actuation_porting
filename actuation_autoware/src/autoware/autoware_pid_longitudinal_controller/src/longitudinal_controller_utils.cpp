// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"

#include <optional>
#include <utility>
#include <algorithm>
#include <limits>
#include <math.h> // isfinite // TODO: check if this is valid

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{

bool isValidTrajectory(const TrajectoryMsg & traj)
{
  auto sequence_points = wrap(traj.points);
  for (const auto & p : sequence_points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.position.z) || !isfinite(p.pose.orientation.w) ||
      !isfinite(p.pose.orientation.x) || !isfinite(p.pose.orientation.y) ||
      !isfinite(p.pose.orientation.z) || !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.acceleration_mps2) ||
      !isfinite(p.heading_rate_rps)) {
      return false;
    }
  }

  // when trajectory is empty
  if (sequence_points.empty()) {
    return false;
  }

  return true;
}

double calcStopDistance(
  const PoseMsg & current_pose, const TrajectoryMsg & traj, const double max_dist, const double max_yaw)
{
  auto sequence_points = wrap(traj.points);
  const auto stop_idx_opt = autoware::motion_utils::searchZeroVelocityIndex(sequence_points);

  const size_t end_idx = stop_idx_opt ? *stop_idx_opt : sequence_points.size() - 1;
  const size_t seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    sequence_points, current_pose, max_dist, max_yaw);
  const double signed_length_on_traj = autoware::motion_utils::calcSignedArcLength(
    sequence_points, current_pose.position, seg_idx, sequence_points.at(end_idx).pose.position,
    std::min(end_idx, sequence_points.size() - 2));

  if (std::isnan(signed_length_on_traj)) {
    return 0.0;
  }
  return signed_length_on_traj;
}

double getPitchByPose(const QuaternionMsg & quaternion_msg)
{
  double roll, pitch, yaw;
  // tf2::Quaternion quaternion;
  // tf2::fromMsg(quaternion_msg, quaternion);
  // tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);
  // TODO: check if this is valid

  // Convert QuaternionMsg to Eigen::Quaterniond
  Eigen::Quaterniond eigen_quat(quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z);
  
  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rotation_matrix = eigen_quat.toRotationMatrix();
  
  // Extract roll, pitch, yaw from rotation matrix
  // Using Eigen's eulerAngles with the XYZ convention
  // Note: Eigen uses different order than ROS, so we extract separately
  roll = atan2(rotation_matrix(2,1), rotation_matrix(2,2));  // roll
  pitch = asin(-rotation_matrix(2,0));                        // pitch
  yaw = atan2(rotation_matrix(1,0), rotation_matrix(0,0));  // yaw
  
  return pitch;
}

double getPitchByTraj(
  const TrajectoryMsg & trajectory, const size_t start_idx, const double wheel_base)
{
  // cannot calculate pitch
  auto sequence_points = wrap(trajectory.points);
  if (sequence_points.size() <= 1) {
    return 0.0;
  }

  const auto [prev_idx, next_idx] = [&]() {
    for (size_t i = start_idx + 1; i < sequence_points.size(); ++i) {
      const double dist = autoware::universe_utils::calcDistance3d(
        sequence_points.at(start_idx), sequence_points.at(i));
      if (dist > wheel_base) {
        // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
        return std::make_pair(start_idx, i);
      }
    }
    // NOTE: The ego pose is close to the goal.
    return std::make_pair(
      std::min(start_idx, sequence_points.size() - 2), sequence_points.size() - 1);
  }();

  return autoware::universe_utils::calcElevationAngle(
    sequence_points.at(prev_idx).pose.position, sequence_points.at(next_idx).pose.position);
}

PoseMsg calcPoseAfterTimeDelay(
  const PoseMsg & current_pose, const double delay_time, const double current_vel,
  const double current_acc)
{
  if (delay_time <= 0.0) {
    return current_pose;
  }

  // check time to stop
  const double time_to_stop = -current_vel / current_acc;

  const double delay_time_calculation =
    time_to_stop > 0.0 && time_to_stop < delay_time ? time_to_stop : delay_time;
  // simple linear prediction
  // TODO: check if this is valid compared to tf2::getYaw
  const double yaw = autoware::universe_utils::getYaw(current_pose.orientation);
  const double running_distance = delay_time_calculation * current_vel + 0.5 * current_acc *
                                                                           delay_time_calculation *
                                                                           delay_time_calculation;
  const double dx = running_distance * std::cos(yaw);
  const double dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

double lerp(const double v_from, const double v_to, const double ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

PoseMsg findTrajectoryPoseAfterDistance(
  const size_t src_idx, const double distance,
  const TrajectoryMsg & trajectory)
{
  double remain_dist = distance;
  auto sequence_points = wrap(trajectory.points);
  PoseMsg p = sequence_points.back().pose;

  for (size_t i = src_idx; i < sequence_points.size() - 1; ++i) {
    const double dist = autoware::universe_utils::calcDistance3d(
      sequence_points.at(i).pose, sequence_points.at(i + 1).pose);
    if (remain_dist < dist) {
      if (remain_dist <= 0.0) {
        return sequence_points.at(i).pose;
      }
      double ratio = remain_dist / dist;
      const auto p0 = sequence_points.at(i).pose;
      const auto p1 = sequence_points.at(i + 1).pose;
      p = sequence_points.at(i).pose;
      p.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      p.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      p.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      p.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
      break;
    }
    remain_dist -= dist;
  }
  return p;
}
}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller
