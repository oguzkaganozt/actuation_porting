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
template void validateNonEmpty<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> &);
template void validateNonEmpty<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> &);
template void validateNonEmpty<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> &);

//
template std::optional<bool> isDrivingForward<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> &);
template std::optional<bool>
isDrivingForward<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> &);
template std::optional<bool>
isDrivingForward<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> &);

//
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> &);

//
template std::vector<autowarePlanningMsgsPathPoint>
removeOverlapPoints<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t start_idx);
template std::vector<tier4PlanningMsgsPathPointWithLaneId>
removeOverlapPoints<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const size_t start_idx);
template std::vector<autowarePlanningMsgsTrajectoryPoint>
removeOverlapPoints<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t start_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const size_t src_idx, const size_t dst_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist);

//
template size_t findNearestIndex<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & point);
template size_t findNearestIndex<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & point);
template size_t findNearestIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & point);

//
template std::optional<size_t>
findNearestIndex<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);

//
template double
calcLongitudinalOffsetToSegment<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t seg_idx,
  const geometryMsgsPoint & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t seg_idx,
  const geometryMsgsPoint & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t seg_idx,
  const geometryMsgsPoint & p_target, const bool throw_exception);

//
template size_t findNearestSegmentIndex<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & point);
template size_t findNearestSegmentIndex<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & point);
template size_t findNearestSegmentIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & point);

//
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);

//
template double calcLateralOffset<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & p_target, const size_t seg_idx, const bool throw_exception);

//
template double calcLateralOffset<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & p_target, const bool throw_exception);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const size_t dst_idx);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

//
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> &,
  const geometryMsgsPoint & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> &,
  const geometryMsgsPoint & src_point, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const geometryMsgsPoint & dst_point);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & src_point, const geometryMsgsPoint & dst_point);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & src_point, const geometryMsgsPoint & dst_point);

//
template double calcArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points);
template double calcArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points);
template double calcArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points);

//
template std::vector<double> calcCurvature<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points);
template std::vector<double>
calcCurvature<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points);
template std::vector<double>
calcCurvature<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points);

//
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points);
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points);
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception);

//
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const double offset);
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & src_point, const double offset);
template std::optional<geometryMsgsPoint>
calcLongitudinalOffsetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & src_point, const double offset);

//
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);

//
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometryMsgsPose>
calcLongitudinalOffsetPose<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & src_point, const double offset,
  const bool set_orientation_from_position_direction);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const size_t seg_idx, const geometryMsgsPoint & p_target,
  std::vector<autowarePlanningMsgsPathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const size_t seg_idx, const geometryMsgsPoint & p_target,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const size_t seg_idx, const geometryMsgsPoint & p_target,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const double insert_point_length, const geometryMsgsPoint & p_target,
  std::vector<autowarePlanningMsgsPathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const double insert_point_length, const geometryMsgsPoint & p_target,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const double insert_point_length, const geometryMsgsPoint & p_target,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autowarePlanningMsgsPathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const geometryMsgsPose & src_pose, const double insert_point_length,
  std::vector<autowarePlanningMsgsPathPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const geometryMsgsPose & src_pose, const double insert_point_length,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const geometryMsgsPose & src_pose, const double insert_point_length,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsPathPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

//
template std::optional<size_t> insertStopPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsPathPoint> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const double distance_to_stop_point,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<autowarePlanningMsgsPathPoint>>(
  const geometryMsgsPose & src_pose, const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsPathPoint> & points_with_twist, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const geometryMsgsPose & src_pose, const double distance_to_stop_point,
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const geometryMsgsPose & src_pose, const double distance_to_stop_point,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const size_t stop_seg_idx, const geometryMsgsPoint & stop_point,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t>
insertDecelPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const geometryMsgsPoint & src_point, const double distance_to_decel_point,
  const double velocity,
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist);

//
template void insertOrientation<std::vector<autowarePlanningMsgsPathPoint>>(
  std::vector<autowarePlanningMsgsPathPoint> & points, const bool is_driving_forward);
template void insertOrientation<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const bool is_driving_forward);
template void insertOrientation<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const bool is_driving_forward);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & src_point, const size_t src_seg_idx, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points, const size_t src_idx,
  const geometryMsgsPoint & dst_point, const size_t dst_seg_idx);

//
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);

//
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPose & pose, const double dist_threshold, const double yaw_threshold);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points_with_twist,
  const geometryMsgsPose & pose, const double max_dist, const double max_yaw);

//
template std::vector<autowarePlanningMsgsPathPoint>
cropForwardPoints<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<tier4PlanningMsgsPathPointWithLaneId>
cropForwardPoints<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<autowarePlanningMsgsTrajectoryPoint>
cropForwardPoints<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length);

//
template std::vector<autowarePlanningMsgsPathPoint>
cropBackwardPoints<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<tier4PlanningMsgsPathPointWithLaneId>
cropBackwardPoints<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<autowarePlanningMsgsTrajectoryPoint>
cropBackwardPoints<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double backward_length);

//
template std::vector<autowarePlanningMsgsPathPoint>
cropPoints<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<tier4PlanningMsgsPathPointWithLaneId>
cropPoints<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<autowarePlanningMsgsTrajectoryPoint>
cropPoints<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);

//
template double calcYawDeviation<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPose & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPose & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPose & pose, const bool throw_exception);

//
template bool isTargetPointFront<std::vector<autowarePlanningMsgsPathPoint>>(
  const std::vector<autowarePlanningMsgsPathPoint> & points,
  const geometryMsgsPoint & base_point, const geometryMsgsPoint & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<tier4PlanningMsgsPathPointWithLaneId>>(
  const std::vector<tier4PlanningMsgsPathPointWithLaneId> & points,
  const geometryMsgsPoint & base_point, const geometryMsgsPoint & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<autowarePlanningMsgsTrajectoryPoint>>(
  const std::vector<autowarePlanningMsgsTrajectoryPoint> & points,
  const geometryMsgsPoint & base_point, const geometryMsgsPoint & target_point,
  const double threshold);

void calculate_time_from_start(
  std::vector<autowarePlanningMsgsTrajectoryPoint> & trajectory,
  const geometryMsgsPoint & current_ego_point, const float min_velocity)
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
