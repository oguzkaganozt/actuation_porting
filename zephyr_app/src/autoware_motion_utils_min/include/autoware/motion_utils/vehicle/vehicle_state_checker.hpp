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

// Zephyr App Includes
#include "zephyr_app.hpp"

// zephyr
// #include "zephyr_app/zephyr_logger.hpp"

namespace autoware::motion_utils
{
class VehicleStopCheckerBase
{
public:
  // VehicleStopCheckerBase(rclcpp::Node * node, double buffer_duration); // TODO: implement with zephyr threads
  VehicleStopCheckerBase(double buffer_duration);
  bool getLogger() { return true; } // TODO: implement with zephyr logger
  void addTwist(const TwistStampedMsg & twist);
  bool isVehicleStopped(const double stop_duration = 0.0) const;

protected:
  // rclcpp::Clock::SharedPtr clock_; // TODO: implement with zephyr clock
  // rclcpp::Logger logger_; // TODO: implement with zephyr logger

private:
  double buffer_duration_;
  std::deque<TwistStampedMsg> twist_buffer_;
};

class VehicleStopChecker : public VehicleStopCheckerBase
{
public:
  // explicit VehicleStopChecker(rclcpp::Node * node); // TODO: implement with zephyr threads
  explicit VehicleStopChecker();

protected:
  // TODO: implement with custom subscription
  // rclcpp::Subscription<OdometryMsg>::SharedPtr sub_odom_;
  // OdometryMsg::ConstSharedPtr odometry_ptr_;

  void* sub_odom_;
  void* odometry_ptr_;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;
  void onOdom(const void* msg);
};

class VehicleArrivalChecker : public VehicleStopChecker
{
public:
  // explicit VehicleArrivalChecker(rclcpp::Node * node); // TODO: implement with zephyr threads
  explicit VehicleArrivalChecker();

  bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const;

private:
  static constexpr double th_arrived_distance_m = 1.0;

  // TODO: implement with custom subscription
  // rclcpp::Subscription<TrajectoryMsg>::SharedPtr sub_trajectory_;
  void* sub_trajectory_;

  // TODO: check shared pointer vs zephyr pointer
  // TrajectoryMsg::ConstSharedPtr trajectory_ptr_;
  void* trajectory_ptr_;

  // TODO: implement with custom callback
  // void onTrajectory(const TrajectoryMsg::ConstSharedPtr msg);
  void onTrajectory(const void* msg);
};
}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
