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

#include <actuation_message_converter/actuation_message_converter_node.hpp>
#include <Trajectory.h>
#include <VehicleControlCommand.h>
#include <VehicleKinematicState.h>

#include <algorithm>
#include <functional>
#include <vector>

namespace autoware
{
namespace actuation_message_converter
{

static void on_command_callback(dds_entity_t rd, void * arg)
{
  if (!arg) {
    return;
  }
  auto publish = *reinterpret_cast<std::function<void(const AckermannControlCommand &)> *>(arg);
  autoware_auto_vehicle_msgs_msg_VehicleControlCommand idlc_msg {};
  void * idlc_msg_p = &idlc_msg;
  dds_sample_info_t info;

  // The return value contains the number of samples read.
  dds_return_t rc = dds_take(rd, &idlc_msg_p, &info, 1, 1);
  if (rc < 0) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "can't take msg\n");
  }

  // Check if we have read some data and if it is valid.
  if (rc > 0 && info.valid_data) {
    AckermannControlCommand rosidl_msg {};

    rosidl_msg.stamp.sec = idlc_msg.stamp.sec;
    rosidl_msg.stamp.nanosec = idlc_msg.stamp.nanosec;

    rosidl_msg.lateral.stamp.sec = idlc_msg.stamp.sec;
    rosidl_msg.lateral.stamp.nanosec = idlc_msg.stamp.nanosec;
    rosidl_msg.lateral.steering_tire_angle = idlc_msg.front_wheel_angle_rad;

    rosidl_msg.longitudinal.stamp.sec = idlc_msg.stamp.sec;
    rosidl_msg.longitudinal.stamp.nanosec = idlc_msg.stamp.nanosec;
    rosidl_msg.longitudinal.speed = idlc_msg.velocity_mps;
    rosidl_msg.longitudinal.acceleration = idlc_msg.long_accel_mps2;

    publish(rosidl_msg);
  }
}

void MessageConverterNode::on_state_callback(const Odometry::SharedPtr rosidl_msg)
{
  autoware_auto_vehicle_msgs_msg_VehicleKinematicState idlc_msg {};
  copy(*rosidl_msg, idlc_msg);
  dds_return_t rc = dds_write(m_state_writer, &idlc_msg);
  if (rc < 0) {
    RCLCPP_ERROR(get_logger(), "dds_write: %s", dds_strretcode(-rc));
  }
}

void MessageConverterNode::on_trajectory_callback(const Trajectory::SharedPtr rosidl_msg)
{
  autoware_auto_planning_msgs_msg_Trajectory idlc_msg {};
  std::vector<autoware_auto_planning_msgs_msg_TrajectoryPoint> buf(
    autoware_auto_planning_msgs_msg_Trajectory_Constants_CAPACITY);
  idlc_msg.points._buffer = buf.data();
  copy(*rosidl_msg, idlc_msg);
  dds_return_t rc = dds_write(m_trajectory_writer, &idlc_msg);
  if (rc < 0) {
    RCLCPP_ERROR(get_logger(), "dds_write: %s", dds_strretcode(-rc));
  }
}

MessageConverterNode::MessageConverterNode(const rclcpp::NodeOptions & options)
: Node("actuation_message_converter", options),
  m_state_sub(create_subscription<Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{1},
      std::bind(&MessageConverterNode::on_state_callback, this, std::placeholders::_1))),
  m_trajectory_sub(create_subscription<Trajectory>(
      "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
      std::bind(&MessageConverterNode::on_trajectory_callback, this, std::placeholders::_1))),
  m_command_pub(create_publisher<AckermannControlCommand>(
      "/control/trajectory_follower/control_cmd", rclcpp::QoS{1}.transient_local()))
{
  // Create a Participant for all DDS usage.
  dds_entity_t participant = dds_create_participant(DDS_DOMAIN_ACTUATION, NULL, NULL);
  if (participant < 0) {
    fprintf(stderr, "dds_create_participant: %s\n", dds_strretcode(-participant));
    std::exit(EXIT_FAILURE);
  }

  // Create a Topic for VehicleControlCommand messages.
  dds_entity_t command_topic = dds_create_topic(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleControlCommand_desc,
    "ctrl_cmd",
    NULL,
    NULL);
  if (command_topic < 0) {
    fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-command_topic));
    std::exit(EXIT_FAILURE);
  }

  // Create a listener.
  m_publish = [&](const AckermannControlCommand & msg) {m_command_pub->publish(msg);};
  m_listener = dds_create_listener(reinterpret_cast<void *>(&m_publish));
  dds_lset_data_available(m_listener, on_command_callback);

  // Create a reliable QoS.
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  // Create a Reader for VehicleControlCommand messages.
  m_command_reader = dds_create_reader(participant, command_topic, qos, m_listener);
  if (m_command_reader < 0) {
    dds_delete_qos(qos);
    fprintf(stderr, "dds_create_reader: %s\n", dds_strretcode(-m_command_reader));
    std::exit(EXIT_FAILURE);
  }
  dds_delete_qos(qos);

  // Create a Topic for VehicleKinematicState messages.
  dds_entity_t state_topic = dds_create_topic(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleKinematicState_desc,
    "current_pose",
    NULL,
    NULL);
  if (state_topic < 0) {
    fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-state_topic));
    std::exit(EXIT_FAILURE);
  }

  // Create a Topic for Trajectory messages.
  dds_entity_t trajectory_topic = dds_create_topic(
    participant,
    &autoware_auto_planning_msgs_msg_Trajectory_desc,
    "trajectory",
    NULL,
    NULL);
  if (trajectory_topic < 0) {
    fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-trajectory_topic));
    std::exit(EXIT_FAILURE);
  }

  // Create a Writer for VehicleKinematicState messages.
  m_state_writer = dds_create_writer(participant, state_topic, NULL, NULL);
  if (m_state_writer < 0) {
    fprintf(stderr, "dds_create_writer: %s\n", dds_strretcode(-m_state_writer));
    std::exit(EXIT_FAILURE);
  }

  // Create a Writer for Trajectory messages.
  m_trajectory_writer = dds_create_writer(participant, trajectory_topic, NULL, NULL);
  if (m_trajectory_writer < 0) {
    fprintf(stderr, "dds_create_writer: %s\n", dds_strretcode(-m_trajectory_writer));
    std::exit(EXIT_FAILURE);
  }
}

MessageConverterNode::~MessageConverterNode()
{
  dds_delete_listener(m_listener);
}

static void copy(const builtin_interfaces::msg::Time & src, builtin_interfaces_msg_Time & dst)
{
  dst.sec = src.sec;
  dst.nanosec = src.nanosec;
}

static void copy(
  const builtin_interfaces::msg::Duration & src,
  builtin_interfaces_msg_Duration & dst)
{
  dst.sec = src.sec;
  dst.nanosec = src.nanosec;
}

static void copy(const std_msgs::msg::Header & src, std_msgs_msg_Header & dst)
{
  copy(src.stamp, dst.stamp);
  // The src message needs to stay alive at least as long as the dst message for a no-copy to work.
  dst.frame_id = const_cast<char *>(src.frame_id.c_str());
}

static void copy(const geometry_msgs::msg::Point & src, geometry_msgs_msg_Point & dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

static void copy(const geometry_msgs::msg::Quaternion & src, geometry_msgs_msg_Quaternion & dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.w = src.w;
}

static void copy(const geometry_msgs::msg::Pose & src, geometry_msgs_msg_Pose & dst)
{
  copy(src.position, dst.position);
  copy(src.orientation, dst.orientation);
}

static void copy(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & src,
  autoware_auto_planning_msgs_msg_TrajectoryPoint & dst)
{
  copy(src.time_from_start, dst.time_from_start);
  copy(src.pose, dst.pose);
  dst.longitudinal_velocity_mps = src.longitudinal_velocity_mps;
  dst.lateral_velocity_mps = src.lateral_velocity_mps;
  dst.acceleration_mps2 = src.acceleration_mps2;
  dst.heading_rate_rps = src.heading_rate_rps;
  dst.front_wheel_angle_rad = src.front_wheel_angle_rad;
  dst.rear_wheel_angle_rad = src.rear_wheel_angle_rad;
}

static void copy(
  const Odometry & src,
  autoware_auto_vehicle_msgs_msg_VehicleKinematicState & dst)
{
  copy(src.header, dst.header);
  dst.state.time_from_start.sec = src.header.stamp.sec;
  dst.state.time_from_start.nanosec = src.header.stamp.nanosec;
  copy(src.pose.pose, dst.state.pose);
  dst.state.longitudinal_velocity_mps = static_cast<float>(src.twist.twist.linear.x);
  dst.state.lateral_velocity_mps = static_cast<float>(src.twist.twist.linear.y);
  dst.state.heading_rate_rps = static_cast<float>(src.twist.twist.angular.z);
}

static void copy(
  const Trajectory::_points_type & src,
  dds_sequence_autoware_auto_planning_msgs_msg_TrajectoryPoint & dst)
{
  dst._maximum = autoware_auto_planning_msgs_msg_Trajectory_Constants_CAPACITY;
  dst._length = std::min(dst._maximum, static_cast<uint32_t>(src.size()));
  for (size_t i = 0; i < dst._length; i++) {
    copy(src[i], dst._buffer[i]);
    // Assume points spaced based on frequency of State publication (40Hz)
    dst._buffer[i].time_from_start.sec = (static_cast<int32_t>(i) + 1) / 40;
    dst._buffer[i].time_from_start.nanosec = ((static_cast<uint32_t>(i) + 1) % 40) * 25000000;
  }
}

static void copy(const Trajectory & src, autoware_auto_planning_msgs_msg_Trajectory & dst)
{
  copy(src.header, dst.header);
  copy(src.points, dst.points);
}

}  // namespace actuation_message_converter
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::actuation_message_converter::MessageConverterNode)
