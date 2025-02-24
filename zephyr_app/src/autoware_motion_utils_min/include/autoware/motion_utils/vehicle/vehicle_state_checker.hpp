// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
#define AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_

// standard library
#include <deque>

// message types
#include "messages.h"

namespace autoware::motion_utils
{

using autowarePlanningMsgsTrajectory = autoware_planning_msgs_msg_Trajectory;
using geometryMsgsTwistStamped = geometry_msgs_msg_TwistStamped;
using navMsgsOdometry = nav_msgs_msg_Odometry;

class VehicleStopCheckerBase
{
public:
  VehicleStopCheckerBase(rclcpp::Node * node, double buffer_duration);
  rclcpp::Logger getLogger() { return logger_; }
  void addTwist(const geometryMsgsTwistStamped & twist);
  bool isVehicleStopped(const double stop_duration = 0.0) const;

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

private:
  double buffer_duration_;
  std::deque<geometryMsgsTwistStamped> twist_buffer_;
};

class VehicleStopChecker : public VehicleStopCheckerBase
{
public:
  explicit VehicleStopChecker(rclcpp::Node * node);

protected:
  rclcpp::Subscription<navMsgsOdometry>::SharedPtr sub_odom_;
  navMsgsOdometry::ConstSharedPtr odometry_ptr_;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;
  void onOdom(const navMsgsOdometry::ConstSharedPtr msg);
};

class VehicleArrivalChecker : public VehicleStopChecker
{
public:
  explicit VehicleArrivalChecker(rclcpp::Node * node);

  bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const;

private:
  static constexpr double th_arrived_distance_m = 1.0;

  rclcpp::Subscription<autowarePlanningMsgsTrajectory>::SharedPtr sub_trajectory_;

  autowarePlanningMsgsTrajectory::ConstSharedPtr trajectory_ptr_;

  void onTrajectory(const autowarePlanningMsgsTrajectory::ConstSharedPtr msg);
};
}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
