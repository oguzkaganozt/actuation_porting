// Copyright 2022 The Autoware Foundation
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_

// Msgs
#include "Trajectory.h"
#include "Odometry.h"
#include "SteeringReport.h"
#include "OperationModeState.h"
#include "AccelWithCovarianceStamped.h"
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using OdometryMsg = nav_msgs_msg_Odometry;
using SteeringReportMsg = autoware_vehicle_msgs_msg_SteeringReport;
using OperationModeStateMsg = autoware_adapi_v1_msgs_msg_OperationModeState;
using AccelWithCovarianceStampedMsg = geometry_msgs_msg_AccelWithCovarianceStamped;

namespace autoware::motion::control::trajectory_follower
{
struct InputData
{
  TrajectoryMsg current_trajectory;
  OdometryMsg current_odometry;
  SteeringReportMsg current_steering;
  AccelWithCovarianceStampedMsg current_accel;
  OperationModeStateMsg current_operation_mode;
};
}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_
