// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY__CONVERSION_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY__CONVERSION_HPP_

#include <vector>

// Msgs
#include "TrajectoryPoint.h"
#include "PathWithLaneId.h"
#include "Path.h"
#include "Trajectory.h"
using TrajectoryPointMsg = autoware_planning_msgs_msg_TrajectoryPoint;
using PathWithLaneIdMsg = tier4_planning_msgs_msg_PathWithLaneId;
using PathMsg = autoware_planning_msgs_msg_Path;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using HeaderMsg = std_msgs_msg_Header;
using PathPointWithLaneIdMsg = tier4_planning_msgs_msg_PathPointWithLaneId;
namespace autoware::motion_utils
{
using TrajectoryPoints = std::vector<TrajectoryPointMsg>;

/**
 * @brief Convert std::vector<autoware_planning_msgs::msg::TrajectoryPoint> to
 * autoware_planning_msgs::msg::Trajectory. This function is temporarily added for porting to
 * autoware_msgs. We should consider whether to remove this function after the porting is done.
 * @attention This function just clips
 * std::vector<autoware_planning_msgs::msg::TrajectoryPoint> up to the capacity of Trajectory.
 * Therefore, the error handling out of this function is necessary if the size of the input greater
 * than the capacity.
 * @todo Decide how to handle the situation that we need to use the trajectory with the size of
 * points larger than the capacity. (Tier IV)
 */
TrajectoryMsg convertToTrajectory(
  const std::vector<TrajectoryPointMsg> & trajectory,
  const HeaderMsg & header = HeaderMsg{});

/**
 * @brief Convert autoware_planning_msgs::msg::Trajectory to
 * std::vector<autoware_planning_msgs::msg::TrajectoryPoint>.
 */
std::vector<TrajectoryPointMsg> convertToTrajectoryPointArray(
  const TrajectoryMsg & trajectory);

template <class T>
PathMsg convertToPath([[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToPath can be used.");
  throw std::logic_error("Only specializations of convertToPath can be used.");
}

template <>
inline PathMsg convertToPath(
  const PathWithLaneIdMsg & input)
{
  PathMsg output{};
  output.header = input.header;
  output.left_bound = input.left_bound;
  output.right_bound = input.right_bound;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

template <class T>
TrajectoryPoints convertToTrajectoryPoints([[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToTrajectoryPoints can be used.");
  throw std::logic_error("Only specializations of convertToTrajectoryPoints can be used.");
}

template <>
inline TrajectoryPoints convertToTrajectoryPoints(
  const PathWithLaneIdMsg & input)
{
  TrajectoryPoints output{};
  for (const auto & p : input.points) {
    TrajectoryPointMsg tp;
    tp.pose = p.point.pose;
    tp.longitudinal_velocity_mps = p.point.longitudinal_velocity_mps;
    // since path point doesn't have acc for now
    tp.acceleration_mps2 = 0;
    output.emplace_back(tp);
  }
  return output;
}

template <class T>
PathWithLaneIdMsg convertToPathWithLaneId([[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToPathWithLaneId can be used.");
  throw std::logic_error("Only specializations of convertToPathWithLaneId can be used.");
}

template <>
inline PathWithLaneIdMsg convertToPathWithLaneId(
  const TrajectoryPoints & input)
{
  PathWithLaneIdMsg output{};
  for (const auto & p : input) {
    PathPointWithLaneIdMsg pp;
    pp.point.pose = p.pose;
    pp.point.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    output.points.emplace_back(pp);
  }
  return output;
}

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY__CONVERSION_HPP_
