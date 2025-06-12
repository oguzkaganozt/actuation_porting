// Copyright 2021 Tier IV, Inc. All rights reserved.
// Edited by: Oguz Ozturk 2025, ARM
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

#include "autoware/trajectory_follower_node/controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"
#include <autoware/trajectory_follower_base/lateral_controller_base.hpp>
#include "common/logger/logger.hpp"
using namespace common::logger;

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(NATIVE_SIM)
static unsigned char node_stack[CONFIG_THREAD_STACK_SIZE];
static unsigned char timer_stack[CONFIG_THREAD_STACK_SIZE];
#else
static K_THREAD_STACK_DEFINE(node_stack, CONFIG_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(timer_stack, CONFIG_THREAD_STACK_SIZE);
#define STACK_SIZE (K_THREAD_STACK_SIZEOF(node_stack))
#endif

namespace
{
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}
}  // namespace

namespace autoware::motion::control::trajectory_follower_node
{
Controller::Controller() : Node("controller", node_stack, STACK_SIZE, timer_stack, STACK_SIZE)
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period", 0.15); // 0.03
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec", 0.5);

  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);

  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter<std::string>("lateral_controller_mode", "mpc"));
  log_debug("Lateral controller mode: %d\n", lateral_controller_mode);
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller_ =
        std::make_shared<mpc_lateral_controller::MpcLateralController>(*this);
      break;
    }
    default:
      log_error("[LateralController] invalid algorithm");
      std::exit(1);
  }

  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter<std::string>("longitudinal_controller_mode", "pid"));
  log_debug("Longitudinal controller mode: %d\n", longitudinal_controller_mode);
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller_ =
        std::make_shared<pid_longitudinal_controller::PidLongitudinalController>(*this);
      break;
    }
    default:
      log_error("[LongitudinalController] invalid algorithm");
      std::exit(1);
  }

  // Timer
  {
    const auto period_ms = ctrl_period*1000;
    create_timer(period_ms, &Controller::callbackTimerControl, this);
  }

  // Subscribers
    auto subscriber_steering_status = create_subscription<SteeringReportMsg>("/vehicle/status/steering_status",
                                                                &autoware_vehicle_msgs_msg_SteeringReport_desc,
                                                                callbackSteeringStatus, this);
    auto subscriber_trajectory = create_subscription<TrajectoryMsg>("/planning/scenario_planning/trajectory",
                                                                &autoware_planning_msgs_msg_Trajectory_desc,
                                                                callbackTrajectory, this);
    auto subscriber_odometry = create_subscription<OdometryMsg>("/localization/kinematic_state",
                                                                &nav_msgs_msg_Odometry_desc,
                                                                callbackOdometry, this);
    auto subscriber_acceleration = create_subscription<AccelWithCovarianceStampedMsg>("/localization/acceleration",
                                                                &geometry_msgs_msg_AccelWithCovarianceStamped_desc,
                                                                callbackAcceleration, this);
    auto subscriber_operation_mode_state = create_subscription<OperationModeStateMsg>("/system/operation_mode/state",
                                                                &autoware_adapi_v1_msgs_msg_OperationModeState_desc,
                                                                callbackOperationModeState, this);
    
  // Publishers
  control_cmd_pub_ = create_publisher<ControlMsg>(
    "~/output/control_cmd", &autoware_control_msgs_msg_Control_desc);
  // pub_processing_time_lat_ms_ =
  //   create_publisher<Float64StampedMsg>("~/lateral/debug/processing_time_ms", &tier4_debug_msgs_msg_Float64Stamped_desc);
  // pub_processing_time_lon_ms_ =
  //   create_publisher<Float64StampedMsg>("~/longitudinal/debug/processing_time_ms", &tier4_debug_msgs_msg_Float64Stamped_desc);
  
  // TODO: we are not publishing these for the sake of simplicity
  // debug_marker_pub_ =
  //   create_publisher<MarkerArrayMsg>("~/output/debug_marker", &visualization_msgs_msg_MarkerArray_desc);
  // if (enable_control_cmd_horizon_pub_) {
  //   control_cmd_horizon_pub_ = create_publisher<ControlHorizonMsg>(
  //     "~/debug/control_cmd_horizon", &autoware_control_msgs_msg_ControlHorizon_desc);
  // }
  // published_time_publisher_ =
  //   std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

// SUBSCRIBER CALLBACKS
void Controller::callbackSteeringStatus(const SteeringReportMsg* msg, void* arg) {
  // static int count = 0;
  // log_debug("-------STEERING STATUS----IDX %d----\n", count++);
  // log_debug("Timestamp: %ld\n", Clock::toDouble(msg->stamp));
  // log_debug("Received steering status: %f\n", msg->steering_tire_angle);
  // log_debug("--------------------------------\n");

  // Put data into state pointers
  Controller* controller = static_cast<Controller*>(arg);
  controller->current_steering_ = *msg;
  controller->has_steering_ = true;
}

void Controller::callbackOperationModeState(const OperationModeStateMsg* msg, void* arg) {
  // static int count = 0;
  // log_debug("-------OPERATION MODE STATE----IDX %d----\n", count++);
  // log_debug("Timestamp: %ld\n", Clock::toDouble(msg->stamp));
  // log_debug("Mode: %d\n", msg->mode);
  // log_debug("Autoware control enabled: %d\n", msg->is_autoware_control_enabled);
  // log_debug("In transition: %d\n", msg->is_in_transition);
  // log_debug("--------------------------------\n");

  // Put data into state pointers
  Controller* controller = static_cast<Controller*>(arg);
  controller->current_operation_mode_ = *msg;
  controller->has_operation_mode_ = true;
}

void Controller::callbackOdometry(const OdometryMsg* msg, void* arg) {
  // static int count = 0;
  // log_debug("-------ODOMETRY----IDX %d----\n", count++);
  // log_debug("Timestamp: %ld\n", Clock::toDouble(msg->stamp));
  // log_debug("Position: %lf, %lf, %lf\n", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  // log_debug("Linear Twist: %lf, %lf, %lf\n", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  // log_debug("-------------------------------\n");

  // Put data into state pointers
  Controller* controller = static_cast<Controller*>(arg);
  controller->current_odometry_ = *msg;
  controller->has_odometry_ = true;
}

void Controller::callbackAcceleration(const AccelWithCovarianceStampedMsg* msg, void* arg) {
  // static int count = 0;
  // log_debug("-------ACCELERATION----IDX %d----\n", count++);
  // log_debug("Timestamp: %ld\n", Clock::toDouble(msg->stamp));
  // log_debug("Linear acceleration: %lf, %lf, %lf\n", msg->accel.accel.linear.x, msg->accel.accel.linear.y, msg->accel.accel.linear.z);
  // log_debug("Angular acceleration: %lf, %lf, %lf\n", msg->accel.accel.angular.x, msg->accel.accel.angular.y, msg->accel.accel.angular.z);
  // log_debug("-------------------------------\n");

  // Put data into state pointers
  Controller* controller = static_cast<Controller*>(arg);
  controller->current_accel_ = *msg;
  controller->has_accel_ = true;
}

void Controller::callbackTrajectory(const TrajectoryMsg* msg, void* arg) {
  static int count = 0;
  log_debug("-------TRAJECTORY----IDX %d----\n", count++);
  log_debug("Timestamp: %f\n", Clock::toDouble(msg->header.stamp));
  log_debug("Trajectory size: %u\n", msg->points._length);
  log_debug("-------------------------------\n");

  if (msg->points._length > 4000) {
    log_warn("Trajectory size is too large: %u\n", msg->points._length);
    return;
  }

  if (msg->points._maximum > 200000) {
    log_warn("Trajectory maximum is too large: %u\n", msg->points._maximum);
    return;
  }

  // Copy the data instead of storing the pointer
  Controller* controller = static_cast<Controller*>(arg);
  controller->current_trajectory_ = *msg;  // Copy the entire message
  controller->has_trajectory_ = true;
}

Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc") return LateralControllerMode::MPC;

  return LateralControllerMode::INVALID;
}

Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;

  return LongitudinalControllerMode::INVALID;
}

bool Controller::processData()
{
  bool is_ready = true;

  const auto & logData = [this](const std::string & data_type) {
    log_info_throttle(("Waiting for " + data_type + " data").c_str());
  };

  if (!has_accel_) {
    logData("acceleration");
    is_ready = false;
  }
  if (!has_steering_) {
    logData("steering");
    is_ready = false;
  }
  if (!has_trajectory_) {
    logData("trajectory");
    is_ready = false;
  }
  if (!has_odometry_) {
    logData("odometry");
    is_ready = false;
  }
  if (!has_operation_mode_) {
    logData("operation mode");
    is_ready = false;
  }

  return is_ready;
}

bool Controller::isTimeOut(
  const trajectory_follower::LongitudinalOutput & lon_out,
  const trajectory_follower::LateralOutput & lat_out)
{
  const auto now = Clock::now();
  if ((now - Clock::toDouble(lat_out.control_cmd.stamp)) > timeout_thr_sec_) {
    log_warn_throttle("Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - Clock::toDouble(lon_out.control_cmd.stamp)) > timeout_thr_sec_) {
    log_warn_throttle("Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
}

std::optional<trajectory_follower::InputData> Controller::createInputData()
{
  if (!processData()) {
    log_info_throttle("Control is skipped since input data is not ready.");
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = current_trajectory_;
  input_data.current_odometry = current_odometry_;
  input_data.current_steering = current_steering_;
  input_data.current_accel = current_accel_;
  input_data.current_operation_mode = current_operation_mode_;

  return input_data;
}

void Controller::callbackTimerControl(void* arg)
{
  Controller* controller = static_cast<Controller*>(arg);

  // 1. create input data
  const auto input_data = controller->createInputData();
  if (!input_data) {
    log_info_throttle("Control is skipped since input data is not ready.");
    return;
  }

  log_debug("Input data created\n");

  // 2. check if controllers are ready
  const bool is_lat_ready = controller->lateral_controller_->isReady(*input_data);
  const bool is_lon_ready = controller->longitudinal_controller_->isReady(*input_data);
  if (!is_lat_ready || !is_lon_ready) {
    log_info_throttle("Control is skipped since lateral and/or longitudinal controllers are not ready to run.");
    return;
  }

  log_debug("Controllers are ready\n");

  // 3. run controllers
  // controller->stop_watch_.tic("lateral");
  const auto lat_out = controller->lateral_controller_->run(*input_data);
  log_debug("-------LAT OUT--\n", 0);
  log_debug("Lateral output: %f\n", lat_out.control_cmd.steering_tire_angle);
  log_debug("Lateral steering tire rotation rate: %f\n", lat_out.control_cmd.steering_tire_rotation_rate);
  log_debug("Lateral is defined steering tire rotation rate: %d\n", lat_out.control_cmd.is_defined_steering_tire_rotation_rate);
  // log_debug("Lateral horizon: %f\n", lat_out.control_cmd_horizon.time_step_ms);
  // log_debug("Lateral sync data: %f\n", lat_out.sync_data.is_steer_converged);
  log_debug("Lateral controller ran\n");
  std::exit(0); // TODO: DEBUG REMOVE
  // controller->stop_watch_.toc("lateral");
  // controller->publishProcessingTime(controller->stop_watch_.toc("lateral"), controller->pub_processing_time_lat_ms_);

  // controller->stop_watch_.tic("longitudinal");
  const auto lon_out = controller->longitudinal_controller_->run(*input_data);
  log_debug("Longitudinal controller ran\n");
  // controller->stop_watch_.toc("longitudinal");
  // controller->publishProcessingTime(controller->stop_watch_.toc("longitudinal"), controller->pub_processing_time_lon_ms_);

  log_debug("Controllers ran\n");
  std::exit(0); // TODO: DEBUG REMOVE

  // 4. sync with each other controllers
  controller->longitudinal_controller_->sync(lat_out.sync_data);
  controller->lateral_controller_->sync(lon_out.sync_data);

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if (controller->isTimeOut(lon_out, lat_out)) return;

  // 5. publish control command
  ControlMsg out;
  out.stamp = Clock::toRosTime(Clock::now());
  out.lateral = lat_out.control_cmd;
  out.longitudinal = lon_out.control_cmd;
  controller->control_cmd_pub_->publish(out);

  //TODO: we are not publishing these for the sake of simplicity
  // // 6. publish debug
  // published_time_publisher_->publish_if_subscribed(control_cmd_pub_, out.stamp);
  // publishDebugMarker(*input_data, lat_out);
  // // 7. publish experimental topic
  // if (enable_control_cmd_horizon_pub_) {
  //   const auto control_horizon =
  //     mergeLatLonHorizon(lat_out.control_cmd_horizon, lon_out.control_cmd_horizon, this->now());
  //   if (control_horizon.has_value()) {
  //     control_cmd_horizon_pub_->publish(control_horizon.value());
  //   }
  // }
}

// TODO: we are not publishing these for the sake of simplicity

// void Controller::publishProcessingTime(
//   const double t_ms, const std::shared_ptr<Publisher<Float64StampedMsg>> pub)
// {
//   Float64StampedMsg msg{};
//   msg.stamp = Clock::toRosTime(Clock::now());
//   msg.data = t_ms;
//   pub->publish(msg);
// }

// void Controller::publishDebugMarker(
//   const trajectory_follower::InputData & input_data,
//   const trajectory_follower::LateralOutput & lat_out) const
// {
//   MarkerArrayMsg debug_marker_array{};

//   // steer converged marker
//   {
//     auto marker = autoware::universe_utils::createDefaultMarker(
//       "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
//       autoware::universe_utils::createMarkerScale(0.0, 0.0, 1.0),
//       autoware::universe_utils::createMarkerColor(1.0, 1.0, 1.0, 0.99));
//     marker.pose = input_data.current_odometry.pose.pose;

//     std::stringstream ss;
//     const double current = input_data.current_steering.steering_tire_angle;
//     const double cmd = lat_out.control_cmd.steering_tire_angle;
//     const double diff = current - cmd;
//     ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
//        << (lat_out.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
//     marker.text = ss.str();

//     debug_marker_array.markers.push_back(marker);
//   }

//   debug_marker_pub_->publish(debug_marker_array);
// }

// std::optional<ControlHorizonMsg> Controller::mergeLatLonHorizon(
//   const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
//   const double & stamp)
// {
//   if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
//     return std::nullopt;
//   }

//   ControlHorizonMsg control_horizon{};
//   control_horizon.stamp = Clock::toRosTime(stamp);

//   // If either of the horizons has only one control, repeat the control to match the other horizon.
//   // lateral horizon
//   if (lateral_horizon.controls.size() == 1) {
//     control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
//     const auto lateral = lateral_horizon.controls.front();
//     for (const auto & longitudinal : longitudinal_horizon.controls) {
//       ControlMsg control;
//       control.longitudinal = longitudinal;
//       control.lateral = lateral;
//       control.stamp = Clock::toRosTime(stamp);
//       auto sequence_controls = wrap_sequence(control_horizon.controls);
//       sequence_controls.push_back(control);
//     }
//     return control_horizon;
//   }

//   // longitudinal horizon
//   if (longitudinal_horizon.controls.size() == 1) {
//     control_horizon.time_step_ms = lateral_horizon.time_step_ms;
//     const auto longitudinal = longitudinal_horizon.controls.front();
//     for (const auto & lateral : lateral_horizon.controls) {
//       ControlMsg control;
//       control.longitudinal = longitudinal;
//       control.lateral = lateral;
//       control.stamp = Clock::toRosTime(stamp);
//       auto sequence_controls = wrap_sequence(control_horizon.controls);
//       sequence_controls.push_back(control);
//     }
//     return control_horizon;
//   }

//   // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
//   // calculate greatest common divisor of time steps
//   const auto gcd_double = [](const double a, const double b) {
//     const double precision = 1e9;
//     const int int_a = static_cast<int>(round(a * precision));
//     const int int_b = static_cast<int>(round(b * precision));
//     return static_cast<double>(std::gcd(int_a, int_b)) / precision;
//   };

//   // calculate greatest common divisor of time steps
//   const double time_step_ms =
//     gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
//   control_horizon.time_step_ms = time_step_ms;

//   // resample lateral horizon
//   const auto lateral_controls = resampleHorizonByZeroOrderHold(
//     lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);

//   // resample longitudinal horizon
//   const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
//     longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

//   // check if sizes match
//   if (lateral_controls.size() != longitudinal_controls.size()) {
//     return std::nullopt;
//   }

//   // merge controls
//   const size_t num_steps = lateral_controls.size();
//   for (size_t i = 0; i < num_steps; ++i) {
//     ControlMsg control{};
//     control.stamp = Clock::toRosTime(stamp);
//     control.lateral = lateral_controls.at(i);
//     control.longitudinal = longitudinal_controls.at(i);
//     auto sequence_controls = wrap_sequence(control_horizon.controls);
//     sequence_controls.push_back(control);
//   }
  
//   return control_horizon;
// }
}  // namespace autoware::motion::control::trajectory_follower_node

