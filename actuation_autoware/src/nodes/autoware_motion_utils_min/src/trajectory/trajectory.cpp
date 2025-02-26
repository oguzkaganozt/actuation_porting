// Copyright 2023 TIER IV, Inc.
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

#include <algorithm>
#include <utility>
#include <vector>

#include "autoware/motion_utils/trajectory/trajectory.hpp"

LOG_MODULE_REGISTER(motion_utils_trajectory);

namespace autoware::motion_utils
{

// validateNonEmpty
template void validateNonEmpty<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> &);
template void validateNonEmpty<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> &);
template void validateNonEmpty<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> &);

// findNearestIndex
template size_t findNearestIndex<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & point);
template size_t findNearestIndex<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & point);
template size_t findNearestIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & point);

// findNearestIndex
template std::optional<size_t>
findNearestIndex<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);

// // isDrivingForward
// template std::optional<bool> isDrivingForward<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> &);
// template std::optional<bool>
// isDrivingForward<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> &);
// template std::optional<bool>
// isDrivingForward<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> &);

// // removeOverlapPoints
// template std::vector<PathPointMsg>
// removeOverlapPoints<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points, const size_t start_idx);
// template std::vector<PathPointWithLaneIdMsg>
// removeOverlapPoints<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points,
//   const size_t start_idx);
// template std::vector<TrajectoryPointMsg>
// removeOverlapPoints<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points, const size_t start_idx);

// // searchZeroVelocityIndex
// template std::optional<size_t>
// searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points_with_twist,
//   const size_t src_idx, const size_t dst_idx);
// template std::optional<size_t>
// searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points_with_twist,
//   const size_t src_idx);
// template std::optional<size_t>
// searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points_with_twist);

// // calcLongitudinalOffsetToSegment
// template double
// calcLongitudinalOffsetToSegment<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points, const size_t seg_idx,
//   const PointMsg & p_target, const bool throw_exception);
// template double
// calcLongitudinalOffsetToSegment<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points, const size_t seg_idx,
//   const PointMsg & p_target, const bool throw_exception);
// template double
// calcLongitudinalOffsetToSegment<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points, const size_t seg_idx,
//   const PointMsg & p_target, const bool throw_exception);

// // findNearestSegmentIndex
// template size_t findNearestSegmentIndex<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points,
//   const PointMsg & point);
// template size_t findNearestSegmentIndex<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points,
//   const PointMsg & point);
// template size_t findNearestSegmentIndex<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points,
//   const PointMsg & point);

// // findNearestSegmentIndex
// template std::optional<size_t>
// findNearestSegmentIndex<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points,
//   const PoseMsg & pose, const double max_dist, const double max_yaw);
// template std::optional<size_t>
// findNearestSegmentIndex<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points,
//   const PoseMsg & pose, const double max_dist, const double max_yaw);
// template std::optional<size_t>
// findNearestSegmentIndex<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points,
//   const PoseMsg & pose, const double max_dist, const double max_yaw);

// // calcSignedArcLength
// template double calcSignedArcLength<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points, const size_t src_idx,
//   const size_t dst_idx);
// template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
//   const size_t dst_idx);
// template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
//   const size_t dst_idx);

// // calcSignedArcLength
// template double calcSignedArcLength<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points,
//   const PointMsg & src_point, const size_t dst_idx);
// template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> &,
//   const PointMsg & src_point, const size_t dst_idx);
// template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> &,
//   const PointMsg & src_point, const size_t dst_idx);

// // calcCurvature
// template std::vector<double> calcCurvature<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points);
// template std::vector<double>
// calcCurvature<std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points);
// template std::vector<double>
// calcCurvature<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points);

// // findFirstNearestIndexWithSoftConstraints
// template size_t
// findFirstNearestIndexWithSoftConstraints<std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
// template size_t findFirstNearestIndexWithSoftConstraints<
//   std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
// template size_t
// findFirstNearestIndexWithSoftConstraints<std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);

// // findFirstNearestSegmentIndexWithSoftConstraints
// template size_t findFirstNearestSegmentIndexWithSoftConstraints<
//   std::vector<PathPointMsg>>(
//   const std::vector<PathPointMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
// template size_t findFirstNearestSegmentIndexWithSoftConstraints<
//   std::vector<PathPointWithLaneIdMsg>>(
//   const std::vector<PathPointWithLaneIdMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
// template size_t findFirstNearestSegmentIndexWithSoftConstraints<
//   std::vector<TrajectoryPointMsg>>(
//   const std::vector<TrajectoryPointMsg> & points,
//   const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);

}  // namespace autoware::motion_utils
