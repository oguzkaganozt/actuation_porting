// Copyright (c) 2022-2023, Arm Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// SPDX-License-Identifier: Apache-2.0

#ifndef ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_
#define ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_

#include <dds/dds.h>

#include <actuation_message_converter/visibility_control.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace actuation_message_converter
{

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using nav_msgs::msg::Odometry;

/// \class MessageConverterNode
/// \brief Converts messages between ROSIDL and IDLC.
class ACTUATION_MESSAGE_CONVERTER_PUBLIC MessageConverterNode : public rclcpp::Node
{
public:
  /// \brief constructor
  /// \param options Node options for this node.
  explicit MessageConverterNode(const rclcpp::NodeOptions & options);

  /// \brief destructor
  ~MessageConverterNode();

  // Domain ID used for Safety Island communication.
  static constexpr dds_domainid_t DDS_DOMAIN_ACTUATION {2};

private:
  const rclcpp::Subscription<Odometry>::SharedPtr m_state_sub;
  const rclcpp::Subscription<Trajectory>::SharedPtr m_trajectory_sub;
  const rclcpp::Publisher<AckermannControlCommand>::SharedPtr m_command_pub;
  dds_listener_t * m_listener {};
  dds_entity_t m_state_writer {};
  dds_entity_t m_trajectory_writer {};
  dds_entity_t m_command_reader {};
  std::function<void(const AckermannControlCommand &)> m_publish {};

  /// \brief Callback handler converting Odometry rosidl messages to VehicleKinematicState idlc
  ///        messages and sending them with a CycloneDDS writer.
  /// \param rosidl_msg Message to be converted.
  void on_state_callback(Odometry::SharedPtr rosidl_msg);

  /// \brief Callback handler converting Trajectory rosidl messages to idlc and sending them with a
  ///        CycloneDDS writer.
  /// \param rosidl_msg Message to be converted.
  void on_trajectory_callback(Trajectory::SharedPtr rosidl_msg);
};
}  // namespace actuation_message_converter
}  // namespace autoware

#endif  // ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_
