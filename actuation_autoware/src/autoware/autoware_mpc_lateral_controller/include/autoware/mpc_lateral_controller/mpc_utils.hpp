// Copyright 2018-2021 The Autoware Foundation
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include "common/clock/clock.hpp"

//Msgs
#include "Trajectory.h"
#include "TrajectoryPoint.h"
#include "PoseStamped.h"
#include "Pose.h"
#include "TwistStamped.h"
using PoseMsg = geometry_msgs_msg_Pose;
using TwistStampedMsg = geometry_msgs_msg_TwistStamped;
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using TrajectoryPointMsg = autoware_planning_msgs_msg_TrajectoryPoint;

namespace autoware::motion::control::mpc_lateral_controller
{
namespace MPCUtils
{

/**
 * @brief calculate 2d distance from trajectory[idx1] to trajectory[idx2]
 */
double calcDistance2d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2);

/**
 * @brief calculate 3d distance from trajectory[idx1] to trajectory[idx2]
 */
double calcDistance3d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2);

/**
 * @brief convert Euler angle vector including +-2pi to 0 jump to continuous series data
 * @param [inout] angle_vector input angle vector
 */
void convertEulerAngleToMonotonic(std::vector<double> & angle_vector);

/**
 * @brief calculate the lateral error of the given pose relative to the given reference pose
 * @param [in] ego_pose pose to check for error
 * @param [in] ref_pose reference pose
 * @return lateral distance between the two poses
 */
double calcLateralError(const PoseMsg & ego_pose, const PoseMsg & ref_pose);

/**
 * @brief convert the given Trajectory msg to a MPCTrajectory object
 * @param [in] input trajectory to convert
 * @return resulting MPCTrajectory
 */
MPCTrajectory convertToMPCTrajectory(const TrajectoryMsg & input);

/**
 * @brief convert the given MPCTrajectory to a Trajectory msg
 * @param [in] input MPCTrajectory to be converted
 * @return output converted Trajectory msg
 */
TrajectoryMsg convertToAutowareTrajectory(const MPCTrajectory & input);

/**
 * @brief calculate the arc length at each point of the given trajectory
 * @param [in] trajectory trajectory for which to calculate the arc length
 * @param [out] arc_length the cumulative arc length at each point of the trajectory
 */
void calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory, std::vector<double> & arc_length);

/**
 * @brief calculate the arc length of the given trajectory
 * @param [in] trajectory trajectory for which to calculate the arc length
 * @return total arc length
 */
double calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory);

/**
 * @brief resample the given trajectory with the given fixed interval
 * @param [in] input trajectory to resample
 * @param [in] resample_interval_dist the desired distance between two successive trajectory points
 * @return The pair contains the successful flag and the resultant resampled trajectory
 */
std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, const size_t nearest_seg_idx,
  const double ego_offset_to_segment);

/**
 * @brief linearly interpolate the given trajectory assuming a base indexing and a new desired
 * indexing
 * @param [in] in_index indexes for each trajectory point
 * @param [in] in_traj MPCTrajectory to interpolate
 * @param [in] out_index desired interpolated indexes
 * @param [out] out_traj resulting interpolated MPCTrajectory
 */
bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory & out_traj);

/**
 * @brief fill the relative_time field of the given MPCTrajectory
 * @param [in] traj MPCTrajectory for which to fill in the relative_time
 * @return true if the calculation was successful
 */
bool calcMPCTrajectoryTime(MPCTrajectory & traj);

/**
 * @brief recalculate the velocity field (vx) of the MPCTrajectory with dynamic smoothing
 * @param [in] start_seg_idx segment index of the trajectory point from which to start smoothing
 * @param [in] start_vel initial velocity to set at the start_seg_idx
 * @param [in] acc_lim limit on the acceleration
 * @param [in] tau constant to control the smoothing (high-value = very smooth)
 * @param [inout] traj MPCTrajectory for which to calculate the smoothed velocity
 */
void dynamicSmoothingVelocity(
  const size_t start_seg_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory & traj);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 * @param [in] shift is forward or not
 */
void calcTrajectoryYawFromXY(MPCTrajectory & traj, const bool is_forward_shift);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3
 * points when num = 1)
 * @param [in] curvature_smoothing_num_traj index distance for 3 points for curvature calculation
 * @param [in] curvature_smoothing_num_ref_steer index distance for 3 points for smoothed curvature
 * calculation
 * @param [inout] traj object trajectory
 */
void calcTrajectoryCurvature(
  const int curvature_smoothing_num_traj, const int curvature_smoothing_num_ref_steer,
  MPCTrajectory & traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3
 * points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [in] traj input trajectory
 * @return vector of curvatures at each point of the given trajectory
 */
std::vector<double> calcTrajectoryCurvature(
  const int curvature_smoothing_num, const MPCTrajectory & traj);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const PoseMsg & self_pose, PoseMsg * nearest_pose,
  size_t * nearest_index, double * nearest_time, const double max_dist, const double max_yaw);

/**
 * @brief calculate distance to stopped point
 */
double calcStopDistance(const TrajectoryMsg & current_trajectory, const int origin);

/**
 * @brief extend terminal points
 * Note: The current MPC does not properly take into account the attitude angle at the end of the
 * path. By extending the end of the path in the attitude direction, the MPC can consider the
 * attitude angle well, resulting in improved control performance. If the trajectory is
 * well-defined considering the end point attitude angle, this feature is not necessary.
 * @param [in] terminal yaw
 * @param [in] extend interval
 * @param [in] flag of forward shift
 * @param [out] extended trajectory
 */
void extendTrajectoryInYawDirection(
  const double yaw, const double interval, const bool is_forward_shift, MPCTrajectory & traj);

/**
 * @brief clip trajectory size by length
 * @param [in] trajectory original trajectory
 * @param [in] length clip length
 * @return clipped trajectory
 */
MPCTrajectory clipTrajectoryByLength(const MPCTrajectory & trajectory, const double length);

void info_throttle(const char * msg);
void warn_throttle(const char * msg);

}  // namespace MPCUtils
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_
