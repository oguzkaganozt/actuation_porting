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

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::motion_utils
{

//
template void validateNonEmpty<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> &);
template void validateNonEmpty<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> &);
template void validateNonEmpty<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> &);

//
template std::optional<bool> isDrivingForward<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> &);
template std::optional<bool>
isDrivingForward<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> &);
template std::optional<bool>
isDrivingForward<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> &);

//
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> &);

//
template std::vector<PathPointMsg>
removeOverlapPoints<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t start_idx);
template std::vector<PathPointWithLaneIdMsg>
removeOverlapPoints<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const size_t start_idx);
template std::vector<TrajectoryPointMsg>
removeOverlapPoints<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t start_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points_with_twist,
  const size_t src_idx, const size_t dst_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points_with_twist,
  const size_t src_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points_with_twist);

//
template size_t findNearestIndex<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & point);
template size_t findNearestIndex<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & point);
template size_t findNearestIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & point);

//
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

//
template double
calcLongitudinalOffsetToSegment<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t seg_idx,
  const PointMsg & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t seg_idx,
  const PointMsg & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t seg_idx,
  const PointMsg & p_target, const bool throw_exception);

//
template size_t findNearestSegmentIndex<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & point);
template size_t findNearestSegmentIndex<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & point);
template size_t findNearestSegmentIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & point);

//
template std::optional<size_t>
findNearestSegmentIndex<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PoseMsg & pose, const double max_dist, const double max_yaw);

//
template double calcLateralOffset<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & p_target, const size_t seg_idx, const bool throw_exception);

//
template double calcLateralOffset<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & p_target, const bool throw_exception);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const size_t dst_idx);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const size_t dst_idx);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const size_t dst_idx);

//
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> &,
  const PointMsg & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> &,
  const PointMsg & src_point, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const PointMsg & dst_point);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const PointMsg & dst_point);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const PointMsg & dst_point);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const PointMsg & dst_point);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & src_point, const PointMsg & dst_point);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & src_point, const PointMsg & dst_point);

//
template double calcArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points);
template double calcArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points);
template double calcArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points);

//
template std::vector<double> calcCurvature<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points);
template std::vector<double>
calcCurvature<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points);
template std::vector<double>
calcCurvature<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points);

//
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points);
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points);
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points_with_twist,
  const size_t src_idx);

//
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const double offset, const bool throw_exception);

//
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const double offset);
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & src_point, const double offset);
template std::optional<PointMsg>
calcLongitudinalOffsetPoint<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & src_point, const double offset);

//
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);

//
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<PoseMsg>
calcLongitudinalOffsetPose<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & src_point, const double offset,
  const bool set_orientation_from_position_direction);

//
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointMsg>>(
  const size_t seg_idx, const PointMsg & p_target,
  std::vector<PathPointMsg> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const size_t seg_idx, const PointMsg & p_target,
  std::vector<PathPointWithLaneIdMsg> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<TrajectoryPointMsg>>(
  const size_t seg_idx, const PointMsg & p_target,
  std::vector<TrajectoryPointMsg> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointMsg>>(
  const double insert_point_length, const PointMsg & p_target,
  std::vector<PathPointMsg> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const double insert_point_length, const PointMsg & p_target,
  std::vector<PathPointWithLaneIdMsg> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<TrajectoryPointMsg>>(
  const double insert_point_length, const PointMsg & p_target,
  std::vector<TrajectoryPointMsg> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointMsg>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<PathPointMsg> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<PathPointWithLaneIdMsg> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<TrajectoryPointMsg>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<TrajectoryPointMsg> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointMsg>>(
  const PoseMsg & src_pose, const double insert_point_length,
  std::vector<PathPointMsg> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<PathPointWithLaneIdMsg>>(
  const PoseMsg & src_pose, const double insert_point_length,
  std::vector<PathPointWithLaneIdMsg> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<TrajectoryPointMsg>>(
  const PoseMsg & src_pose, const double insert_point_length,
  std::vector<TrajectoryPointMsg> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<PathPointMsg>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<PathPointMsg> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<PathPointWithLaneIdMsg>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<PathPointWithLaneIdMsg> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<TrajectoryPointMsg>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<TrajectoryPointMsg> & points_with_twist,
  const double overlap_threshold = 1e-3);

//
template std::optional<size_t> insertStopPoint<std::vector<PathPointMsg>>(
  const double distance_to_stop_point,
  std::vector<PathPointMsg> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<PathPointWithLaneIdMsg>>(
  const double distance_to_stop_point,
  std::vector<PathPointWithLaneIdMsg> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<TrajectoryPointMsg>>(
  const double distance_to_stop_point,
  std::vector<TrajectoryPointMsg> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<PathPointMsg>>(
  const PoseMsg & src_pose, const double distance_to_stop_point,
  std::vector<PathPointMsg> & points_with_twist, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<PathPointWithLaneIdMsg>>(
  const PoseMsg & src_pose, const double distance_to_stop_point,
  std::vector<PathPointWithLaneIdMsg> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<TrajectoryPointMsg>>(
  const PoseMsg & src_pose, const double distance_to_stop_point,
  std::vector<TrajectoryPointMsg> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<TrajectoryPointMsg>>(
  const size_t stop_seg_idx, const PointMsg & stop_point,
  std::vector<TrajectoryPointMsg> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t>
insertDecelPoint<std::vector<TrajectoryPointMsg>>(
  const PointMsg & src_point, const double distance_to_decel_point,
  const double velocity,
  std::vector<TrajectoryPointMsg> & points_with_twist);

//
template void insertOrientation<std::vector<PathPointMsg>>(
  std::vector<PathPointMsg> & points, const bool is_driving_forward);
template void insertOrientation<std::vector<PathPointWithLaneIdMsg>>(
  std::vector<PathPointWithLaneIdMsg> & points,
  const bool is_driving_forward);
template void insertOrientation<std::vector<TrajectoryPointMsg>>(
  std::vector<TrajectoryPointMsg> & points,
  const bool is_driving_forward);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & src_point, const size_t src_seg_idx, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points, const size_t src_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points, const size_t src_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points, const size_t src_idx,
  const PointMsg & dst_point, const size_t dst_seg_idx);

//
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);

//
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PoseMsg & pose, const double dist_threshold, const double yaw_threshold);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points_with_twist,
  const PoseMsg & pose, const double max_dist, const double max_yaw);

//
template std::vector<PathPointMsg>
cropForwardPoints<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<PathPointWithLaneIdMsg>
cropForwardPoints<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<TrajectoryPointMsg>
cropForwardPoints<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length);

//
template std::vector<PathPointMsg>
cropBackwardPoints<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<PathPointWithLaneIdMsg>
cropBackwardPoints<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<TrajectoryPointMsg>
cropBackwardPoints<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double backward_length);

//
template std::vector<PathPointMsg>
cropPoints<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<PathPointWithLaneIdMsg>
cropPoints<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<TrajectoryPointMsg>
cropPoints<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);

//
template double calcYawDeviation<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PoseMsg & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PoseMsg & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PoseMsg & pose, const bool throw_exception);

//
template bool isTargetPointFront<std::vector<PathPointMsg>>(
  const std::vector<PathPointMsg> & points,
  const PointMsg & base_point, const PointMsg & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<PathPointWithLaneIdMsg>>(
  const std::vector<PathPointWithLaneIdMsg> & points,
  const PointMsg & base_point, const PointMsg & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<TrajectoryPointMsg>>(
  const std::vector<TrajectoryPointMsg> & points,
  const PointMsg & base_point, const PointMsg & target_point,
  const double threshold);

void calculate_time_from_start(
  std::vector<TrajectoryPointMsg> & trajectory,
  const PointMsg & current_ego_point, const float min_velocity)
{
  const auto nearest_segment_idx = findNearestSegmentIndex(trajectory, current_ego_point);
  if (nearest_segment_idx + 1 == trajectory.size()) {
    return;
  }
  for (auto & p : trajectory) {
    p.time_from_start = rclcpp::Duration::from_seconds(0);
  }
  // TODO(Maxime): some points can have very low velocities which introduce huge time errors
  // Temporary solution: use a minimum velocity
  for (auto idx = nearest_segment_idx + 1; idx < trajectory.size(); ++idx) {
    const auto & from = trajectory[idx - 1];
    const auto velocity = std::max(min_velocity, from.longitudinal_velocity_mps);
    if (velocity != 0.0) {
      auto & to = trajectory[idx];
      const auto t = universe_utils::calcDistance2d(from, to) / velocity;
      to.time_from_start = rclcpp::Duration::from_seconds(t) + from.time_from_start;
    }
  }
}
}  // namespace autoware::motion_utils
