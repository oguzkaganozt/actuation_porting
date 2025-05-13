// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware/trajectory_follower_node/visibility_control.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#ifndef NATIVE_SIM
#define fabsl(x) fabs(x)  //TODO:Check compatibility
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/node/node.hpp"

//Msgs
// #include <autoware_control_msgs/msg/detail/control_horizon__struct.hpp> //TODO: check if we are using this
#include "Control.h"
#include "ControlHorizon.h"
#include "Longitudinal.h"
#include "Trajectory.h"
#include "AccelStamped.h"
#include "AccelWithCovarianceStamped.h"
#include "PoseStamped.h"
#include "Odometry.h"
#include "MarkerArray.h"
#include "Float64Stamped.h"
#include "OperationModeState.h"
#include "SteeringReport.h"
#include "TrajectoryPoint.h"
#include "OperationModeState.h"
using OperationModeStateMsg = autoware_adapi_v1_msgs_msg_OperationModeState;
using ControlHorizonMsg = autoware_control_msgs_msg_ControlHorizon;
using Float64StampedMsg = tier4_debug_msgs_msg_Float64Stamped;
using MarkerArrayMsg = visualization_msgs_msg_MarkerArray;
using ControlMsg = autoware_control_msgs_msg_Control;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using OdometryMsg = nav_msgs_msg_Odometry;
using SteeringReportMsg = autoware_vehicle_msgs_msg_SteeringReport;
using AccelWithCovarianceStampedMsg = geometry_msgs_msg_AccelWithCovarianceStamped;
using TrajectoryPointMsg = autoware_planning_msgs_msg_TrajectoryPoint;
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;

namespace autoware::motion::control
{
using trajectory_follower::LateralHorizon;
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalHorizon;
using trajectory_follower::LongitudinalOutput;

namespace trajectory_follower_node
{

using autoware::universe_utils::StopWatch;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public Node
{
public:
  explicit Controller();
  virtual ~Controller() {}

private:
  double timeout_thr_sec_;
  bool enable_control_cmd_horizon_pub_{false};
  std::optional<LongitudinalOutput> longitudinal_output_{std::nullopt};

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  // Subscribers
  //TODO: we don't need to store the subscribers as lifetime of the node is same as the application
  static void callbackSteeringStatus(SteeringReportMsg& msg);
  static void callbackOperationModeState(OperationModeStateMsg& msg);
  static void callbackOdometry(OdometryMsg& msg);
  static void callbackAcceleration(AccelerationMsg& msg);
  static void callbackTrajectory(TrajectoryMsg& msg);

  // Publishers
  std::shared_ptr<Publisher<ControlMsg>> control_cmd_pub_;
  std::shared_ptr<Publisher<Float64StampedMsg>> pub_processing_time_lat_ms_;
  std::shared_ptr<Publisher<Float64StampedMsg>> pub_processing_time_lon_ms_;
  std::shared_ptr<Publisher<ControlHorizonMsg>> control_cmd_horizon_pub_;
  // std::shared_ptr<Publisher<MarkerArrayMsg>> debug_marker_pub_;
  
  // State
  std::shared_ptr<TrajectoryMsg> current_trajectory_ptr_;
  std::shared_ptr<OdometryMsg> current_odometry_ptr_;
  std::shared_ptr<SteeringReportMsg> current_steering_ptr_;
  std::shared_ptr<AccelWithCovarianceStampedMsg> current_accel_ptr_;
  std::shared_ptr<OperationModeStateMsg> current_operation_mode_ptr_;

  enum class LateralControllerMode {
    INVALID = 0,
    MPC = 1,
    PURE_PURSUIT = 2,
  };
  enum class LongitudinalControllerMode {
    INVALID = 0,
    PID = 1,
  };

  /**
   * @brief compute control command, and publish periodically
   */
  std::optional<trajectory_follower::InputData> createInputData();

  //
  static void callbackTimerControl(void* arg);

  //
  bool processData();

  //
  bool isTimeOut(const LongitudinalOutput & lon_out, const LateralOutput & lat_out);

  //
  LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;

  //
  LongitudinalControllerMode getLongitudinalControllerMode(
    const std::string & algorithm_name) const;

  //
  void publishDebugMarker(
    const trajectory_follower::InputData & input_data,
    const trajectory_follower::LateralOutput & lat_out) const;

  //
  /**
   * @brief merge lateral and longitudinal horizons
   * @details If one of the commands has only one control, repeat the control to match the other
   *          horizon. If each horizon has different time intervals, resample them to match the size
   *          with the greatest common divisor.
   * @param lateral_horizon lateral horizon
   * @param longitudinal_horizon longitudinal horizon
   * @param stamp stamp
   * @return merged control horizon
   */
  static std::optional<ControlHorizonMsg> mergeLatLonHorizon(
    const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
    const double & stamp);

  //
  void publishProcessingTime(
    const double t_ms, const std::shared_ptr<Publisher<Float64StampedMsg>> pub);

  //
  StopWatch<std::chrono::milliseconds> stop_watch_;
};

}  // namespace trajectory_follower_node
}  // namespace autoware::motion::control

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
