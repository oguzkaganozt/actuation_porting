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

#include <pthread.h>
#include <gtest/gtest.h>
#include <dds/dds.h>

#include <atomic>
#include <cstdlib>
#include <memory>

#include <Trajectory.h>
#include <VehicleKinematicState.h>
#include <VehicleControlCommand.h>

#include <actuation_message_converter/actuation_message_converter_node.hpp>
#include <rclcpp/rclcpp.hpp>

using autoware::actuation_message_converter::MessageConverterNode;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using namespace std::chrono_literals;

static std::atomic<bool> received_trajectory {false};
static dds_entity_t command_writer {};

static bool get_msg(dds_entity_t rd, void * sample)
{
  dds_sample_info_t info;
  dds_return_t rc = dds_take(rd, &sample, &info, 1, 1);
  EXPECT_GE(rc, 0);
  if (rc > 0 && info.valid_data) {
    return true;
  }
  return false;
}

static void on_trajectory(dds_entity_t rd, void * arg)
{
  (void) arg;
  autoware_auto_planning_msgs_msg_Trajectory msg {};
  bool valid = get_msg(rd, reinterpret_cast<void *>(&msg));
  if (valid) {
    received_trajectory.store(true);
  }
}

static void on_state(dds_entity_t rd, void * arg)
{
  (void) arg;
  autoware_auto_vehicle_msgs_msg_VehicleKinematicState msg {};
  bool valid = get_msg(rd, reinterpret_cast<void *>(&msg));
  if (valid && received_trajectory.load()) {
    autoware_auto_vehicle_msgs_msg_VehicleControlCommand command_msg;
    command_msg.long_accel_mps2 = msg.state.acceleration_mps2;
    command_msg.velocity_mps = msg.state.longitudinal_velocity_mps;
    command_msg.front_wheel_angle_rad = msg.state.front_wheel_angle_rad;
    command_msg.rear_wheel_angle_rad = msg.state.rear_wheel_angle_rad;
    dds_write(command_writer, &command_msg);
  }
}

TEST(MessageConverter, Translation) {
  ASSERT_FALSE(rclcpp::ok());
  rclcpp::init(0, nullptr);
  ASSERT_TRUE(rclcpp::ok());

  std::promise<AckermannControlCommand> command_received {};

  // The ROS2 node, acting as the main compute side.
  auto ros_node = std::make_shared<rclcpp::Node>("node");
  auto trajectory_pub = ros_node->create_publisher<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{5});
  auto state_pub = ros_node->create_publisher<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{5});
  auto command_sub = ros_node->create_subscription<AckermannControlCommand>(
    "/control/trajectory_follower/control_cmd", rclcpp::QoS{5},
    [&](const AckermannControlCommand::SharedPtr msg) {command_received.set_value(*msg);});

  // The message converter.
  auto message_converter = std::make_shared<MessageConverterNode>(rclcpp::NodeOptions());

  // The DDS readers and writers, acting as the actuation side.
  constexpr auto domain = MessageConverterNode::DDS_DOMAIN_ACTUATION;
  dds_entity_t participant = dds_create_participant(domain, NULL, NULL);
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  dds_entity_t trajectory_topic = dds_create_topic(
    participant,
    &autoware_auto_planning_msgs_msg_Trajectory_desc,
    "trajectory",
    NULL,
    NULL
  );
  dds_listener_t * trajectory_listener = dds_create_listener(NULL);
  dds_lset_data_available(trajectory_listener, on_trajectory);
  dds_create_reader(participant, trajectory_topic, qos, trajectory_listener);

  dds_entity_t state_topic = dds_create_topic(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleKinematicState_desc,
    "current_pose",
    NULL,
    NULL
  );
  dds_listener_t * state_listener = dds_create_listener(NULL);
  dds_lset_data_available(state_listener, on_state);
  dds_create_reader(participant, state_topic, qos, state_listener);

  dds_entity_t command_topic = dds_create_topic(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleControlCommand_desc,
    "ctrl_cmd",
    NULL,
    NULL
  );
  command_writer = dds_create_writer(participant, command_topic, qos, NULL);

  dds_delete_qos(qos);

  // Send State and Trajectory messages, expect a Command back.
  rclcpp::executors::SingleThreadedExecutor executor {};
  executor.add_node(ros_node);
  executor.add_node(message_converter);

  Trajectory trajectory_out {};
  TrajectoryPoint point_out {};
  trajectory_out.points.push_back(point_out);

  Odometry state_out {};
  state_out.twist.twist.linear.x = 1.234F;
  state_out.twist.twist.angular.z = -0.123F;

  Odometry empty_state_out {};
  state_pub->publish(empty_state_out);
  executor.spin_some();
  trajectory_pub->publish(trajectory_out);
  executor.spin_some();
  state_pub->publish(state_out);

  auto command_received_future = command_received.get_future().share();
  auto spin_result = executor.spin_until_future_complete(command_received_future, 100ms);
  if (spin_result == rclcpp::FutureReturnCode::SUCCESS) {
    auto msg_in = command_received_future.get();
    EXPECT_EQ(msg_in.longitudinal.speed, state_out.twist.twist.linear.x);
  } else {
    FAIL() << "timeout";
  }

  dds_delete_listener(trajectory_listener);
  dds_delete_listener(state_listener);
  rclcpp::shutdown();
}
