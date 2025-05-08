//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include "common/logger/logger.hpp"
using namespace common::logger;

#include <algorithm>

#if defined(NATIVE_SIM)
#define STACK_SIZE (4096)
static unsigned char node_stack[STACK_SIZE];
static unsigned char timer_stack[STACK_SIZE];
#else
static K_THREAD_STACK_DEFINE(node_stack, 4096);
static K_THREAD_STACK_DEFINE(timer_stack, 4096);
#define STACK_SIZE (K_THREAD_STACK_SIZEOF(node_stack))
#endif

namespace simple_trajectory_follower
{

using autoware::motion_utils::findNearestIndex;
using autoware::universe_utils::calcLateralDeviation;
using autoware::universe_utils::calcYawDeviation;

SimpleTrajectoryFollower::SimpleTrajectoryFollower()
: Node("simple_trajectory_follower", node_stack, STACK_SIZE, timer_stack, STACK_SIZE)
{
  pub_cmd_ = create_publisher<ControlMsg>("output/control_cmd", &autoware_control_msgs_msg_Control_desc);

  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel", true);
  external_target_vel_ = declare_parameter<double>("external_target_vel", 5.0);  //TODO: changed from float to double
  lateral_deviation_ = declare_parameter<double>("lateral_deviation", 0.0);  //TODO: changed from float to double

  //TODO: check for 30ms correctness
  create_timer(0.03, &SimpleTrajectoryFollower::onTimer, this);
}

void SimpleTrajectoryFollower::onTimer(void* arg)
{
  log_info("onTimer\n");  //TODO: remove
  auto self = static_cast<SimpleTrajectoryFollower*>(arg);

  if (!self->processData()) {
    log_error("data not ready\n");
    return;
  }

  self->updateClosest();

  ControlMsg cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = Clock::toRosTime(Clock::now());
  cmd.lateral.steering_tire_angle = static_cast<float>(self->calcSteerCmd());
  cmd.longitudinal.velocity = self->use_external_target_vel_
                                ? static_cast<float>(self->external_target_vel_)
                                : self->closest_traj_point_.longitudinal_velocity_mps;
  cmd.longitudinal.acceleration = static_cast<float>(self->calcAccCmd());
  self->pub_cmd_->publish(cmd);
}

void SimpleTrajectoryFollower::updateClosest()
{
  //TODO: check wrap
  auto sequence_traj_points_ = wrap(trajectory_->points);
  const auto closest = findNearestIndex(sequence_traj_points_, odometry_->pose.pose.position);
  closest_traj_point_ = sequence_traj_points_.at(closest);
}

double SimpleTrajectoryFollower::calcSteerCmd()
{
  const auto lat_err =
    calcLateralDeviation(closest_traj_point_.pose, odometry_->pose.pose.position) -
    lateral_deviation_;
  const auto yaw_err = calcYawDeviation(closest_traj_point_.pose, odometry_->pose.pose);

  // linearized pure_pursuit control
  constexpr auto wheel_base = 4.0;
  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry_->twist.twist.linear.x);
  const auto kp = 2.0 * wheel_base / (lookahead * lookahead);
  const auto kd = 2.0 * wheel_base / lookahead;

  constexpr auto steer_lim = 0.6;

  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim, steer_lim);
  log_info("kp = %f, lat_err = %f, kd = %f, yaw_err = %f, steer = %f\n", kp, lat_err, kd, yaw_err, steer);
  return steer;
}

double SimpleTrajectoryFollower::calcAccCmd()
{
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  log_info("vel_err = %f, acc = %f\n", vel_err, acc);
  return acc;
}

//TODO: check as with other processData methods
bool SimpleTrajectoryFollower::processData()
{
  bool is_ready = true;
  // const auto & getData = [](auto & dest, auto & sub) {
  //   const auto temp = sub.takeData();
  //   if (!temp) return false;
  //   dest = temp;
  //   return true;
  // };
  // is_ready &= getData(odometry_, sub_kinematics_);
  // is_ready &= getData(trajectory_, sub_trajectory_);
  return is_ready;
}

}  // namespace simple_trajectory_follower

