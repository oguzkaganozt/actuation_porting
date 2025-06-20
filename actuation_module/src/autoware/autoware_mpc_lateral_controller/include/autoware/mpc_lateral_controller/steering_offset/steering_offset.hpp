// Copyright 2018-2023 The Autoware Foundation
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_OFFSET__STEERING_OFFSET_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_OFFSET__STEERING_OFFSET_HPP_

#include <cmath>
#include <deque>
#include <vector>

#include "autoware/autoware_msgs/messages.hpp"

class SteeringOffsetEstimator
{
public:
  SteeringOffsetEstimator(
    double wheelbase, double average_num, double vel_thres, double steer_thres,
    double offset_limit);
  ~SteeringOffsetEstimator() = default;

  double getOffset() const;
  void updateOffset(const TwistMsg & twist, const double steering);

private:
  // parameters
  double wheelbase_ = 3.0;
  size_t average_num_ = 1000;
  double update_vel_threshold_ = 8.0;
  double update_steer_threshold_ = 0.05;
  double offset_limit_ = 0.02;

  // results
  std::deque<double> steering_offset_storage_;
  double steering_offset_ = 0.0;
};

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_OFFSET__STEERING_OFFSET_HPP_
