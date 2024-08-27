// Copyright (c) 2023, Arm Limited.
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
//
// SPDX-License-Identifier: Apache-2.0
//

#ifndef ACTUATION_PLAYER__ACTUATION_PLAYER_HPP_
#define ACTUATION_PLAYER__ACTUATION_PLAYER_HPP_

#include <dds/dds.h>

#include <Trajectory.h>
#include <VehicleKinematicState.h>

#define DDS_DOMAIN_ACTUATION 2

using TrajectoryPoint = autoware_auto_planning_msgs_msg_TrajectoryPoint;
using Trajectory = autoware_auto_planning_msgs_msg_Trajectory;
using KinematicState = autoware_auto_vehicle_msgs_msg_VehicleKinematicState;

/// @brief Player component. Setup a DDS participant and start sending messages from the data in
///        recorded files.
/// @param path_prefix Path to find the recorded files
/// @param run_loop Whether the replay runs in a loop or not
/// @param replay_divider Divider of the replay speed
/// @param[in] cancel Pointer to a cancellation flag
/// @return Error code
int actuation_player(
  const char * path_prefix, bool run_loop, float replay_divider, volatile bool * cancel = nullptr);

#endif  // ACTUATION_PLAYER__ACTUATION_PLAYER_HPP_
