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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__MSG_OPERATION_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__MSG_OPERATION_HPP_

#include "autoware/universe_utils/messages.h"

// NOTE: Do not use autoware_universe_utils namespace
namespace geometry_msgs
{
namespace msg
{
geometryMsgsQuaternion operator+(geometryMsgsQuaternion a, geometryMsgsQuaternion b) noexcept;
geometryMsgsQuaternion operator-(geometryMsgsQuaternion a) noexcept;
geometryMsgsQuaternion operator-(geometryMsgsQuaternion a, geometryMsgsQuaternion b) noexcept;
}  // namespace msg
}  // namespace geometry_msgs

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__MSG_OPERATION_HPP_
