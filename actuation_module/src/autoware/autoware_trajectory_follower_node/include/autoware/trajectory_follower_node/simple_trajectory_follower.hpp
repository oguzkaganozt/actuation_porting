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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_

#include <memory>

#include "common/node/node.hpp"
#include "common/clock/clock.hpp"

//Msgs
#include "Control.h"
#include "Trajectory.h"
#include "TrajectoryPoint.h"
#include "Pose.h"
#include "Twist.h"
#include "Odometry.h"
using ControlMsg = autoware_control_msgs_msg_Control;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using TrajectoryPointMsg = autoware_planning_msgs_msg_TrajectoryPoint;
using PoseMsg = geometry_msgs_msg_Pose;
using TwistMsg = geometry_msgs_msg_Twist;
using OdometryMsg = nav_msgs_msg_Odometry;

namespace simple_trajectory_follower
{

class SimpleTrajectoryFollower : public Node
{
public:
  explicit SimpleTrajectoryFollower();
  ~SimpleTrajectoryFollower() = default;

private:
  //TODO: check validity with polling subscribers
  std::shared_ptr<Subscriber<OdometryMsg>> sub_kinematics_;
  std::shared_ptr<Subscriber<TrajectoryMsg>> sub_trajectory_;
  std::shared_ptr<Publisher<ControlMsg>> pub_cmd_;

  std::shared_ptr<TrajectoryMsg> trajectory_;
  std::shared_ptr<OdometryMsg> odometry_;
  TrajectoryPointMsg closest_traj_point_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double lateral_deviation_;

  static void onTimer(void* arg);
  bool processData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
};

}  // namespace simple_trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
