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

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"

#include "common/logger/logger.hpp"
using namespace common::logger;

namespace autoware::motion::control::mpc_lateral_controller
{
using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::normalizeRadian;
using autoware::universe_utils::rad2deg;

MPC::MPC(Node & node)
{
}

ResultWithReason MPC::calculateMPC(
  const SteeringReportMsg & current_steer, const OdometryMsg & current_kinematics, LateralMsg & ctrl_cmd,
  TrajectoryMsg & predicted_trajectory, LateralHorizon & ctrl_cmd_horizon)
{
  log_debug("MPC: Start Calculating");

  // since the reference trajectory does not take into account the current velocity of the ego
  // vehicle, it needs to calculate the trajectory velocity considering the longitudinal dynamics.
  const auto reference_trajectory =
    applyVelocityDynamicsFilter(m_reference_trajectory, current_kinematics);

  log_debug("MPC: Reference Trajectory Applied");

  // get the necessary data
  const auto [get_data_result, mpc_data] =
    getData(reference_trajectory, current_steer, current_kinematics);
  if (!get_data_result.result) {
    return ResultWithReason{false, std::string("getting MPC Data (") + get_data_result.reason + std::string(").")};
  }

  log_debug("MPC: Data Got");

  // calculate initial state of the error dynamics
  const auto x0 = getInitialState(mpc_data);

  // apply time delay compensation to the initial state
  const auto [success_delay, x0_delayed] =
    updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, x0);
  if (!success_delay) {
    log_error("MPC: Delay Compensation Failed");
    return ResultWithReason{false, std::string("delay compensation.")};
  }

  log_debug("MPC: Delay Compensation Applied");

  // resample reference trajectory with mpc sampling time
  const double mpc_start_time = mpc_data.nearest_time + m_param.input_delay;
  const double prediction_dt =
    getPredictionDeltaTime(mpc_start_time, reference_trajectory, current_kinematics);

  const auto [resample_result, mpc_resampled_ref_trajectory] =
    resampleMPCTrajectoryByTime(mpc_start_time, prediction_dt, reference_trajectory);
  if (!resample_result.result) {
    log_error("MPC: Trajectory Resampling Failed");
    return ResultWithReason{
      false, std::string("trajectory resampling (") + resample_result.reason + std::string(").")};
  }
  
  log_debug("MPC: Resampled Reference Trajectory Size: %zu", mpc_resampled_ref_trajectory.size());

  // TODO: POSSIBLE EIGEN ALIGNMENT PROBLEM
  // generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex
  const auto mpc_matrix = generateMPCMatrix(mpc_resampled_ref_trajectory, prediction_dt);

  log_debug("MPC: MPC Matrix Generated");

  // TODO: POSSIBLE EIGEN ALIGNMENT PROBLEM
  // solve Optimization problem
  const auto [opt_result, Uex] = executeOptimization(
    mpc_matrix, x0_delayed, prediction_dt, mpc_resampled_ref_trajectory,
    current_kinematics.twist.twist.linear.x);
  if (!opt_result.result) {
    return ResultWithReason{false, std::string("optimization failure (") + opt_result.reason + std::string(").")};
  }

  log_debug("MPC: Optimization Problem Solved");

  // apply filters for the input limitation and low pass filter
  const double u_saturated = std::clamp(Uex(0), -m_steer_lim, m_steer_lim);
  const double u_filtered = m_lpf_steering_cmd.filter(u_saturated);

  log_debug("MPC: Input Limitation Applied");

  // set control command
  ctrl_cmd.steering_tire_angle = static_cast<float>(u_filtered);
  ctrl_cmd.steering_tire_rotation_rate = static_cast<float>(calcDesiredSteeringRate(
    mpc_matrix, x0_delayed, Uex, u_filtered, current_steer.steering_tire_angle, prediction_dt));

  log_debug("MPC: Control Command Set");

  // save the control command for the steering prediction
  m_steering_predictor->storeSteerCmd(u_filtered);

  // save input to buffer for delay compensation
  m_input_buffer.push_back(ctrl_cmd.steering_tire_angle);
  m_input_buffer.pop_front();

  // save previous input for the mpc rate limit
  m_raw_steer_cmd_pprev = m_raw_steer_cmd_prev;
  m_raw_steer_cmd_prev = Uex(0);

  log_debug("MPC: Control Command Stored");

  // TODO: REMOVED FOR SIMPLIFICATION
  // /* calculate predicted trajectory */
  // Eigen::VectorXd initial_state = m_use_delayed_initial_state ? x0_delayed : x0;
  // predicted_trajectory = calculatePredictedTrajectory(
  //   mpc_matrix, initial_state, Uex, mpc_resampled_ref_trajectory, prediction_dt, "world");

  // // Publish predicted trajectories in different coordinates for debugging purposes
  // if (m_publish_debug_trajectories) {
  //   // Calculate and publish predicted trajectory in Frenet coordinate
  //   auto predicted_trajectory_frenet = calculatePredictedTrajectory(
  //     mpc_matrix, initial_state, Uex, mpc_resampled_ref_trajectory, prediction_dt, "frenet");
  //   predicted_trajectory_frenet.header.stamp = Clock::toRosTime(Clock::now());
  //   predicted_trajectory_frenet.header.frame_id = "map";
  //   m_debug_frenet_predicted_trajectory_pub->publish(predicted_trajectory_frenet);
  // }

  // create LateralHorizon command
  ctrl_cmd_horizon.time_step_ms = prediction_dt * 1000.0;
  ctrl_cmd_horizon.controls.clear();
  ctrl_cmd_horizon.controls.push_back(ctrl_cmd);
  for (auto it = std::next(Uex.begin()); it != Uex.end(); ++it) {
    LateralMsg lateral{};
    lateral.steering_tire_angle = static_cast<float>(std::clamp(*it, -m_steer_lim, m_steer_lim));
    lateral.steering_tire_rotation_rate =
      (lateral.steering_tire_angle - ctrl_cmd_horizon.controls.back().steering_tire_angle) /
      m_ctrl_period;
    ctrl_cmd_horizon.controls.push_back(lateral);
  }

  log_debug("MPC: Lateral Horizon Command Set");
  
  return ResultWithReason{true};
}

void MPC::setReferenceTrajectory(
  const TrajectoryMsg & trajectory_msg, const TrajectoryFilteringParam & param,
  const OdometryMsg & current_kinematics)
{
  const size_t nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      trajectory_msg.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  const double ego_offset_to_segment = autoware::motion_utils::calcLongitudinalOffsetToSegment(
    trajectory_msg.points, nearest_seg_idx, current_kinematics.pose.pose.position);

  const auto mpc_traj_raw = MPCUtils::convertToMPCTrajectory(trajectory_msg);

  // resampling
  const auto [success_resample, mpc_traj_resampled] = MPCUtils::resampleMPCTrajectoryByDistance(
    mpc_traj_raw, param.traj_resample_dist, nearest_seg_idx, ego_offset_to_segment);
  if (!success_resample) {
    log_warn_throttle("[setReferenceTrajectory] spline error when resampling by distance");
    return;
  }

  const auto is_forward_shift =
    autoware::motion_utils::isDrivingForward(mpc_traj_resampled.toTrajectoryPoints());

  // if driving direction is unknown, use previous value
  m_is_forward_shift = is_forward_shift ? is_forward_shift.value() : m_is_forward_shift;

  // path smoothing
  MPCTrajectory mpc_traj_smoothed = mpc_traj_resampled;  // smooth filtered trajectory
  const int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (
    param.enable_path_smoothing && mpc_traj_resampled_size > 2 * param.path_filter_moving_ave_num) {
    using MoveAverageFilter::filt_vector;
    if (
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.x) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.y) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.yaw) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.vx)) {
      log_error("MPC: path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /*
   * Extend terminal points
   * Note: The current MPC does not properly take into account the attitude angle at the end of the
   * path. By extending the end of the path in the attitude direction, the MPC can consider the
   * attitude angle well, resulting in improved control performance. If the trajectory is
   * well-defined considering the end point attitude angle, this feature is not necessary.
   */
  if (param.extend_trajectory_for_end_yaw_control) {
    MPCUtils::extendTrajectoryInYawDirection(
      mpc_traj_raw.yaw.back(), param.traj_resample_dist, m_is_forward_shift, mpc_traj_smoothed);
  }

  // calculate yaw angle
  MPCUtils::calcTrajectoryYawFromXY(mpc_traj_smoothed, m_is_forward_shift);
  MPCUtils::convertEulerAngleToMonotonic(mpc_traj_smoothed.yaw);

  // calculate curvature
  MPCUtils::calcTrajectoryCurvature(
    param.curvature_smoothing_num_traj, param.curvature_smoothing_num_ref_steer, mpc_traj_smoothed);

  // stop velocity at a terminal point
  mpc_traj_smoothed.vx.back() = 0.0;

  // add a extra point on back with extended time to make the mpc stable.
  auto last_point = mpc_traj_smoothed.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  mpc_traj_smoothed.push_back(last_point);

  if (!mpc_traj_smoothed.size()) {
    log_error("MPC: path callback: trajectory size is undesired.");
    return;
  }

  m_reference_trajectory = mpc_traj_smoothed;
}

void MPC::resetPrevResult(const SteeringReportMsg & current_steer)
{
  // Consider limit. The prev value larger than limitation breaks the optimization constraint and
  // results in optimization failure.
  const float steer_lim_f = static_cast<float>(m_steer_lim);
  m_raw_steer_cmd_prev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
  m_raw_steer_cmd_pprev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
}

std::pair<ResultWithReason, MPCData> MPC::getData(
  const MPCTrajectory & traj, const SteeringReportMsg & current_steer,
  const OdometryMsg & current_kinematics)
{
  log_debug("MPC: Getting Data");

  const auto current_pose = current_kinematics.pose.pose;

  MPCData data;
  if (!MPCUtils::calcNearestPoseInterp(
        traj, current_pose, &(data.nearest_pose), &(data.nearest_idx), &(data.nearest_time),
        ego_nearest_dist_threshold, ego_nearest_yaw_threshold)) {
    return {ResultWithReason{false, "error in calculating nearest pose"}, MPCData{}};
  }

  log_debug("MPC: Nearest Pose Calculated");

  // get data
  data.steer = static_cast<double>(current_steer.steering_tire_angle);
  data.lateral_err = MPCUtils::calcLateralError(current_pose, data.nearest_pose);
  data.yaw_err = normalizeRadian(
    autoware::universe_utils::getYaw(current_pose.orientation) - autoware::universe_utils::getYaw(data.nearest_pose.orientation));

  // get predicted steer
  data.predicted_steer = m_steering_predictor->calcSteerPrediction();

  log_debug("MPC: Predicted Steer Calculated");

  // check error limit
  const double dist_err = calcDistance2d(current_pose, data.nearest_pose);
  if (dist_err > m_admissible_position_error) {
    return {ResultWithReason{false, "too large position error"}, MPCData{}};
  }

  log_debug("MPC: Position Error Checked");

  // check yaw error limit
  if (std::fabs(data.yaw_err) > m_admissible_yaw_error_rad) {
    return {ResultWithReason{false, "too large yaw error"}, MPCData{}};
  }

  log_debug("MPC: Yaw Error Checked");

  // check trajectory time length
  const double max_prediction_time =
    m_param.min_prediction_length / static_cast<double>(m_param.prediction_horizon - 1);
  auto end_time = data.nearest_time + m_param.input_delay + m_ctrl_period + max_prediction_time;
  if (end_time > traj.relative_time.back()) {
    return {ResultWithReason{false, "path is too short for prediction."}, MPCData{}};
  }

  log_debug("MPC: Trajectory Time Length Checked");

  return {ResultWithReason{true}, data};
}

std::pair<ResultWithReason, MPCTrajectory> MPC::resampleMPCTrajectoryByTime(
  const double ts, const double prediction_dt, const MPCTrajectory & input) const
{
  MPCTrajectory output;
  std::vector<double> mpc_time_v;
  for (double i = 0; i < static_cast<double>(m_param.prediction_horizon); ++i) {
    mpc_time_v.push_back(ts + i * prediction_dt);
  }
  if (!MPCUtils::linearInterpMPCTrajectory(input.relative_time, input, mpc_time_v, output)) {
    return {ResultWithReason{false, "mpc resample error"}, {}};
  }
  // Publish resampled reference trajectory for debug purpose.
  // if (m_publish_debug_trajectories) {
  //   auto converted_output = MPCUtils::convertToAutowareTrajectory(output);
  //   converted_output.header.stamp = Clock::toRosTime(Clock::now());
  //   converted_output.header.frame_id = "map";
  //   m_debug_resampled_reference_trajectory_pub->publish(converted_output);
  // }
  return {ResultWithReason{true}, output};
}

VectorXd MPC::getInitialState(const MPCData & data)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  VectorXd x0 = VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err;
  const auto & steer = m_use_steer_prediction ? data.predicted_steer : data.steer;
  const auto & yaw_err = data.yaw_err;

  const auto vehicle_model = m_vehicle_model_ptr->modelName();
  if (vehicle_model == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model == "dynamics") {
    double dlat = (lat_err - m_lateral_error_prev) / m_ctrl_period;
    double dyaw = (yaw_err - m_yaw_error_prev) / m_ctrl_period;
    m_lateral_error_prev = lat_err;
    m_yaw_error_prev = yaw_err;
    dlat = m_lpf_lateral_error.filter(dlat);
    dyaw = m_lpf_yaw_error.filter(dyaw);
    x0 << lat_err, dlat, yaw_err, dyaw;
    log_info("MPC: (before lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
    log_info("MPC: (after lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
  } else {
    log_error("MPC: vehicle_model_type is undefined");
  }
  return x0;
}

std::pair<bool, VectorXd> MPC::updateStateForDelayCompensation(
  const MPCTrajectory & traj, const double & start_time, const VectorXd & x0_orig)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);

  const double sign_vx = m_is_forward_shift ? 1 : -1;

  MatrixXd x_curr = x0_orig;
  double mpc_curr_time = start_time;
  for (size_t i = 0; i < m_input_buffer.size(); ++i) {
    double k, v = 0.0;
    try {
      // NOTE: When driving backward, the curvature's sign should be reversed.
      k = autoware::interpolation::lerp(traj.relative_time, traj.k, mpc_curr_time) * sign_vx;
      v = autoware::interpolation::lerp(traj.relative_time, traj.vx, mpc_curr_time);
    } catch (const std::exception & e) {
      log_error("MPC: mpc resample failed at delay compensation, stop mpc: %s", e.what());
      return {false, {}};
    }

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(v);
    m_vehicle_model_ptr->setCurvature(k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, m_ctrl_period);
    MatrixXd ud = MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = m_input_buffer.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += m_ctrl_period;
  }
  return {true, x_curr};
}

MPCTrajectory MPC::applyVelocityDynamicsFilter(
  const MPCTrajectory & input, const OdometryMsg & current_kinematics) const
{
  log_debug("MPC: Applying Velocity Dynamics Filter");

  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);
  if (autoware_traj.points.empty()) {
    return input;
  }

  const size_t nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);

  log_debug("MPC: Nearest Segment Index Found");

  MPCTrajectory output = input;
  MPCUtils::dynamicSmoothingVelocity(
    nearest_seg_idx, current_kinematics.twist.twist.linear.x, m_param.acceleration_limit,
    m_param.velocity_time_constant, output);

  log_debug("MPC: Dynamic Smoothing Velocity Applied");

  auto last_point = output.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  output.push_back(last_point);

  log_debug("MPC: Velocity Dynamics Filter Applied");

  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCMatrix MPC::generateMPCMatrix(
  const MPCTrajectory & reference_trajectory, const double prediction_dt)
{
  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  MPCMatrix m;
  m.Aex = MatrixXd::Zero(DIM_X * N, DIM_X);
  m.Bex = MatrixXd::Zero(DIM_X * N, DIM_U * N);
  m.Wex = MatrixXd::Zero(DIM_X * N, 1);
  m.Cex = MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  m.Qex = MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  m.R1ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.R2ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.Uref_ex = MatrixXd::Zero(DIM_U * N, 1);

  // weight matrix depends on the vehicle model
  MatrixXd Q = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R = MatrixXd::Zero(DIM_U, DIM_U);
  MatrixXd Q_adaptive = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R_adaptive = MatrixXd::Zero(DIM_U, DIM_U);

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);
  MatrixXd Uref(DIM_U, 1);

  const double sign_vx = m_is_forward_shift ? 1 : -1;

  // predict dynamics for N times
  for (int i = 0; i < N; ++i) {
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_vx_squared = ref_vx * ref_vx;

    // NOTE: When driving backward, the curvature's sign should be reversed.
    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double ref_smooth_k = reference_trajectory.smooth_k.at(i) * sign_vx;

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(ref_vx);
    m_vehicle_model_ptr->setCurvature(ref_k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = MatrixXd::Zero(DIM_Y, DIM_Y);
    R = MatrixXd::Zero(DIM_U, DIM_U);
    const auto mpc_weight = getWeight(ref_k);
    Q(0, 0) = mpc_weight.lat_error;
    Q(1, 1) = mpc_weight.heading_error;
    R(0, 0) = mpc_weight.steering_input;

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = m_param.nominal_weight.terminal_lat_error;
      Q_adaptive(1, 1) = m_param.nominal_weight.terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * mpc_weight.heading_error_squared_vel;
    R_adaptive(0, 0) += ref_vx_squared * mpc_weight.steering_input_squared_vel;

    // update mpc matrix
    int idx_x_i = i * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      m.Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      m.Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      m.Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      int idx_x_i_prev = (i - 1) * DIM_X;
      m.Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * m.Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        m.Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * m.Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      m.Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * m.Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    m.Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    m.Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    m.Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    m.R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    // get reference input (feed-forward)
    m_vehicle_model_ptr->setCurvature(ref_smooth_k);
    m_vehicle_model_ptr->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < autoware::universe_utils::deg2rad(m_param.zero_ff_steer_deg)) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Uref_ex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  // add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double j = ref_vx * ref_vx * getWeight(ref_k).lat_jerk / (DT * DT);
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(prediction_dt, m.R1ex);

  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
std::pair<ResultWithReason, VectorXd> MPC::executeOptimization(
  const MPCMatrix & m, const VectorXd & x0, const double prediction_dt, const MPCTrajectory & traj,
  const double current_velocity)
{
  log_debug("MPC: Executing Optimization");
  VectorXd Uex;

  if (!isValid(m)) {
    log_error("MPC: Invalid Model Matrix");
    return {ResultWithReason{false, "invalid model matrix"}, {}};
  }

  log_debug("MPC: Model Matrix Valid");

  const int DIM_U_N = m_param.prediction_horizon * m_vehicle_model_ptr->getDimU();

  // TODO: POSSIBLE EIGEN ALIGNMENT PROBLEM
  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const MatrixXd CB = m.Cex * m.Bex;
  const MatrixXd QCB = m.Qex * CB;
  // MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for
  // a good way.  //NOLINT
  // TODO: POSSIBLE EIGEN ALIGNMENT PROBLEM
  MatrixXd H = MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  MatrixXd f = (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Uref_ex.transpose() * m.R1ex;
  addSteerWeightF(prediction_dt, f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  log_debug("MPC: Cost Function Set");

  // steering angle limit
  VectorXd lb = VectorXd::Constant(DIM_U_N, -m_steer_lim);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, m_steer_lim);   // max steering angle

  log_debug("MPC: Steering Angle Limit Set");

  // steering angle rate limit
  VectorXd steer_rate_limits = calcSteerRateLimitOnTrajectory(traj, current_velocity);
  VectorXd ubA = steer_rate_limits * prediction_dt;
  VectorXd lbA = -steer_rate_limits * prediction_dt;
  ubA(0) = m_raw_steer_cmd_prev + steer_rate_limits(0) * m_ctrl_period;
  lbA(0) = m_raw_steer_cmd_prev - steer_rate_limits(0) * m_ctrl_period;

  log_debug("MPC: Steering Angle Rate Limit Set");

  // TODO: POSSIBLE EIGEN ALIGNMENT PROBLEM
  auto t_start = Clock::now();
  bool solve_result = m_qpsolver_ptr->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex);
  auto t_end = Clock::now();
  if (!solve_result) {
    log_error("MPC: QP Solver Error");
    return {ResultWithReason{false, "qp solver error"}, {}};
  }

  log_debug("MPC: QP Solver Finished");

  {
    auto t = t_end - t_start;
    log_info("MPC: qp solver calculation time = %ld [ms]", t);
  }

  if (Uex.array().isNaN().any()) {
    log_error("MPC: Model Uex including NaN");
    return {ResultWithReason{false, "model Uex including NaN"}, {}};
  }

  log_debug("MPC: Optimization Finished");

  return {ResultWithReason{true}, Uex};
}

void MPC::addSteerWeightR(const double prediction_dt, MatrixXd & R) const
{
  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;

  // add steering rate : weight for (u(i) - u(i-1) / dt )^2
  {
    const double steer_rate_r = m_param.nominal_weight.steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += m_param.nominal_weight.steer_rate / (m_ctrl_period * m_ctrl_period);
    }
  }

  // add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2
  {
    const double w = m_param.nominal_weight.steer_acc;
    const double steer_acc_r = w / std::pow(DT, 4);
    const double steer_acc_r_cp1 = w / (std::pow(DT, 3) * m_ctrl_period);
    const double steer_acc_r_cp2 = w / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
    const double steer_acc_r_cp4 = w / std::pow(m_ctrl_period, 4);
    const Eigen::Matrix3d D =
      steer_acc_r *
      (Eigen::Matrix3d() << 1.0, -2.0, 1.0, -2.0, 4.0, -2.0, 1.0, -2.0, 1.0).finished();
    for (int i = 1; i < N - 1; ++i) {
      R.block(i - 1, i - 1, 3, 3) += D;
    }
    if (N > 1) {
      // steer acc i = 1
      R(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
      R(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(1, 1) += steer_acc_r * 1.0;
      // steer acc i = 0
      R(0, 0) += steer_acc_r_cp4 * 1.0;
    }
  }
}

void MPC::addSteerWeightF(const double prediction_dt, MatrixXd & f) const
{
  if (f.rows() < 2) {
    return;
  }

  const double DT = prediction_dt;

  // steer rate for i = 0
  f(0, 0) += -2.0 * m_param.nominal_weight.steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = m_param.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 3) * m_ctrl_period);
  const double steer_acc_r_cp2 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
  const double steer_acc_r_cp4 = m_param.nominal_weight.steer_acc / std::pow(m_ctrl_period, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * m_raw_steer_cmd_prev + m_raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * m_raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * m_raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

double MPC::getPredictionDeltaTime(
  const double start_time, const MPCTrajectory & input, const OdometryMsg & current_kinematics) const
{
  // Calculate the time min_prediction_length ahead from current_pose
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);

  const size_t nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  double sum_dist = 0;
  const double target_time = [&]() {
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    for (size_t i = nearest_idx + 1; i < input.relative_time.size(); i++) {
      const double segment_dist = MPCUtils::calcDistance2d(input, i, i - 1);
      sum_dist += segment_dist;
      if (m_param.min_prediction_length < sum_dist) {
        const double prev_sum_dist = sum_dist - segment_dist;
        const double ratio = (m_param.min_prediction_length - prev_sum_dist) / segment_dist;
        const double relative_time_at_i = i == input.relative_time.size() - 1
                                            ? input.relative_time.at(i) - t_ext
                                            : input.relative_time.at(i);
        return input.relative_time.at(i - 1) +
               (relative_time_at_i - input.relative_time.at(i - 1)) * ratio;
      }
    }
    return input.relative_time.back() - t_ext;
  }();

  // Calculate delta time for min_prediction_length
  const double dt =
    (target_time - start_time) / static_cast<double>(m_param.prediction_horizon - 1);

  return std::max(dt, m_param.prediction_dt);
}

double MPC::calcDesiredSteeringRate(
  const MPCMatrix & mpc_matrix, const MatrixXd & x0, const MatrixXd & Uex, const double u_filtered,
  const float current_steer, const double predict_dt) const
{
  if (m_vehicle_model_ptr->modelName() != "kinematics") {
    // not supported yet. Use old implementation.
    return (u_filtered - current_steer) / predict_dt;
  }

  // calculate predicted states to get the steering motion
  const auto & m = mpc_matrix;
  const MatrixXd Xex = m.Aex * x0 + m.Bex * Uex + m.Wex;

  const size_t STEER_IDX = 2;  // for kinematics model

  const auto steer_0 = x0(STEER_IDX, 0);
  const auto steer_1 = Xex(STEER_IDX, 0);

  const auto steer_rate = (steer_1 - steer_0) / predict_dt;

  return steer_rate;
}

VectorXd MPC::calcSteerRateLimitOnTrajectory(
  const MPCTrajectory & trajectory, const double current_velocity) const
{
  const auto interp = [&](const auto & steer_rate_limit_map, const auto & current) {
    std::vector<double> reference, limits;
    for (const auto & p : steer_rate_limit_map) {
      reference.push_back(p.first);
      limits.push_back(p.second);
    }

    // If the speed is out of range of the reference, apply zero-order hold.
    if (current <= reference.front()) {
      return limits.front();
    }
    if (current >= reference.back()) {
      return limits.back();
    }

    // Apply linear interpolation
    for (size_t i = 0; i < reference.size() - 1; ++i) {
      if (reference.at(i) <= current && current <= reference.at(i + 1)) {
        auto ratio =
          (current - reference.at(i)) / std::max(reference.at(i + 1) - reference.at(i), 1.0e-5);
        ratio = std::clamp(ratio, 0.0, 1.0);
        const auto interp = limits.at(i) + ratio * (limits.at(i + 1) - limits.at(i));
        return interp;
      }
    }

    log_error("MPC::calcSteerRateLimitOnTrajectory() interpolation logic is broken. Command "
                    "filter is not working. Please check the code.");
    return reference.back();
  };

  // When the vehicle is stopped, a large steer rate limit is used for the dry steering.
  constexpr double steer_rate_lim = 100.0;
  const bool is_vehicle_stopped = std::fabs(current_velocity) < 0.01;
  if (is_vehicle_stopped) {
    return steer_rate_lim * VectorXd::Ones(m_param.prediction_horizon);
  }

  // calculate steering rate limit
  VectorXd steer_rate_limits = VectorXd::Zero(m_param.prediction_horizon);
  for (int i = 0; i < m_param.prediction_horizon; ++i) {
    const auto limit_by_curvature = interp(m_steer_rate_lim_map_by_curvature, trajectory.k.at(i));
    const auto limit_by_velocity = interp(m_steer_rate_lim_map_by_velocity, trajectory.vx.at(i));
    steer_rate_limits(i) = std::min(limit_by_curvature, limit_by_velocity);
  }

  return steer_rate_limits;
}

TrajectoryMsg MPC::calculatePredictedTrajectory(
  const MPCMatrix & mpc_matrix, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
  const MPCTrajectory & reference_trajectory, const double dt, const std::string & coordinate) const
{
  MPCTrajectory predicted_mpc_trajectory;

  if (coordinate == "world") {
    predicted_mpc_trajectory = m_vehicle_model_ptr->calculatePredictedTrajectoryInWorldCoordinate(
      mpc_matrix.Aex, mpc_matrix.Bex, mpc_matrix.Cex, mpc_matrix.Wex, x0, Uex, reference_trajectory,
      dt);
  } else if (coordinate == "frenet") {
    predicted_mpc_trajectory = m_vehicle_model_ptr->calculatePredictedTrajectoryInFrenetCoordinate(
      mpc_matrix.Aex, mpc_matrix.Bex, mpc_matrix.Cex, mpc_matrix.Wex, x0, Uex, reference_trajectory,
      dt);
  } else {
    log_error("Invalid coordinate system specified. Use 'world' or 'frenet'.");
    std::exit(1);
  }

  // do not over the reference trajectory
  const auto predicted_length = MPCUtils::calcMPCTrajectoryArcLength(reference_trajectory);
  const auto clipped_trajectory =
    MPCUtils::clipTrajectoryByLength(predicted_mpc_trajectory, predicted_length);

  const auto predicted_trajectory = MPCUtils::convertToAutowareTrajectory(clipped_trajectory);

  return predicted_trajectory;
}

bool MPC::isValid(const MPCMatrix & m) const
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Uref_ex.array().isNaN().any()) {
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Uref_ex.array().isInf().any()) {
    return false;
  }

  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
