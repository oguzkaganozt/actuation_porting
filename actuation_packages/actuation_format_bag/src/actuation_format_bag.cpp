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

// SPDX-License-Identifier: Apache-2.0

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

namespace
{
/// @brief Maximum number of points to write in a trajectory.
constexpr size_t TRAJ_MAX_POINTS = 100;

/// @brief Write Trajectory message to file.
/// @param msg Message to write
/// @param file File to write to
/// @param ts Timestamp
void write_msg(const Trajectory & msg, std::ofstream & file, uint64_t ts)
{
  // Limit the number of points in the trajectory.
  uint32_t nb_points = std::min(msg.points.size(), TRAJ_MAX_POINTS);

  // Write trajectory header
  file << ts;
  file << ',' << nb_points;

  // Write trajectory points
  for (size_t i = 0; i < nb_points; i++) {
    file << ',' << msg.points[i].pose.position.x;
    file << ',' << msg.points[i].pose.position.y;
    file << ',' << msg.points[i].pose.position.z;
    file << ',' << msg.points[i].pose.orientation.x;
    file << ',' << msg.points[i].pose.orientation.y;
    file << ',' << msg.points[i].pose.orientation.z;
    file << ',' << msg.points[i].pose.orientation.w;
    file << ',' << msg.points[i].longitudinal_velocity_mps;
    file << ',' << msg.points[i].lateral_velocity_mps;
    file << ',' << msg.points[i].acceleration_mps2;
    file << ',' << msg.points[i].heading_rate_rps;
    file << ',' << msg.points[i].front_wheel_angle_rad;
    file << ',' << msg.points[i].rear_wheel_angle_rad;
  }
  file << std::endl;
}

/// @brief Write Odometry message to file.
/// @param msg Message to write
/// @param file File to write to
/// @param ts Timestamp
void write_msg(const Odometry & msg, std::ofstream & file, uint64_t ts)
{
  file << ts;
  file << ',' << msg.pose.pose.position.x;
  file << ',' << msg.pose.pose.position.y;
  file << ',' << msg.pose.pose.position.z;
  file << ',' << msg.pose.pose.orientation.x;
  file << ',' << msg.pose.pose.orientation.y;
  file << ',' << msg.pose.pose.orientation.z;
  file << ',' << msg.pose.pose.orientation.w;
  file << ',' << static_cast<float>(msg.twist.twist.linear.x);
  file << ',' << static_cast<float>(msg.twist.twist.linear.y);
  file << ',' << 0.0;
  file << ',' << static_cast<float>(msg.twist.twist.angular.z);
  file << ',' << 0.0;
  file << ',' << 0.0;
  file << std::endl;
}

/// @brief Write AckermannControlCommand message to file.
/// @param msg Message to write
/// @param file File to write to
/// @param ts Timestamp
void write_msg(const AckermannControlCommand & msg, std::ofstream & file, uint64_t ts)
{
  file << ts;
  file << ',' << msg.longitudinal.acceleration;
  file << ',' << msg.longitudinal.speed;
  file << ',' << msg.lateral.steering_tire_angle;
  file << ',' << 0.0;
  file << std::endl;
}

/// @brief Deserialize message.
/// @tparam MsgT Type of the message to write
/// @param ser_msg Rosbag message
/// @return Deserialized message
template<typename MsgT>
MsgT deserialize(const rosbag2_storage::SerializedBagMessage & ser_msg)
{
  rclcpp::Serialization<MsgT> serialization {};

  MsgT msg;
  rclcpp::SerializedMessage serialized(*ser_msg.serialized_data);
  serialization.deserialize_message(&serialized, &msg);

  return msg;
}

/// @brief Write rosbag message to file.
/// @tparam MsgT Type of the message to write
/// @param ser_msg Rosbag message
/// @param file File to write to
template<typename MsgT>
void write_ser_msg(
  const rosbag2_storage::SerializedBagMessage & ser_msg,
  std::ofstream & file)
{
  // Skip messages with no data
  if (ser_msg.serialized_data) {
    write_msg(deserialize<MsgT>(ser_msg), file, ser_msg.time_stamp);
  }
}

/// @brief Check equality of AckermannControlCommand content.
/// @param ser_a First message to compare
/// @param ser_b Second message to compare
/// @return True when equal
bool equal_cmd(
  const rosbag2_storage::SerializedBagMessage & ser_a,
  const rosbag2_storage::SerializedBagMessage & ser_b)
{
  auto a {deserialize<AckermannControlCommand>(ser_a)};
  auto b {deserialize<AckermannControlCommand>(ser_b)};

  return a.lateral.steering_tire_angle == b.lateral.steering_tire_angle &&
         a.lateral.steering_tire_rotation_rate == b.lateral.steering_tire_rotation_rate &&
         a.longitudinal.speed == b.longitudinal.speed &&
         a.longitudinal.acceleration == b.longitudinal.acceleration &&
         a.longitudinal.jerk == b.longitudinal.jerk;
}

/// @brief Write messages to file.
/// @param rosbag_path Path to the rosbag file
/// @param traj_file File to write Trajectory messages
/// @param kine_file File to write Odometry messages
/// @param cmd_file File to write AckermannControlCommand messages
/// @return Error code
int write(
  const char * rosbag_path,
  std::ofstream & traj_file,
  std::ofstream & kine_file,
  std::ofstream & cmd_file)
{
  rosbag2_cpp::Reader reader {};
  try {
    reader.open(rosbag_path);
  } catch (...) {
    std::fprintf(stderr, "Error opening rosbag\n");
    return 1;
  }

  const std::string traj_topic = "/planning/scenario_planning/trajectory";
  const std::string kine_topic = "/localization/kinematic_state";
  const std::string cmd_topic = "/control/trajectory_follower/control_cmd";

  reader.set_filter({{traj_topic.c_str(), kine_topic.c_str(), cmd_topic.c_str()}});

  rosbag2_storage::SerializedBagMessage current_traj {};
  rosbag2_storage::SerializedBagMessage previous_traj {};
  rosbag2_storage::SerializedBagMessage current_kine {};
  rosbag2_storage::SerializedBagMessage previous_kine {};
  rosbag2_storage::SerializedBagMessage current_cmd {};
  rosbag2_storage::SerializedBagMessage previous_cmd {};

  // Skip to first trajectory message
  while (reader.has_next()) {
    auto ser_msg = reader.read_next();
    if (ser_msg->topic_name == traj_topic) {
      current_traj = *ser_msg;
      break;
    }
  }

  // Skip to first unique command message
  while (reader.has_next()) {
    auto ser_msg = reader.read_next();
    if (ser_msg->topic_name == traj_topic) {
      previous_traj = current_traj;
      current_traj = *ser_msg;
    } else if (ser_msg->topic_name == kine_topic) {
      previous_kine = current_kine;
      current_kine = *ser_msg;
    } else if (ser_msg->topic_name == cmd_topic) {
      previous_cmd = current_cmd;
      current_cmd = *ser_msg;
      // On the first unique command
      if (previous_cmd.serialized_data && !equal_cmd(previous_cmd, current_cmd)) {
        // Write the previous trajectory if the current one is newer than the first kinematic state
        if (current_traj.time_stamp > previous_kine.time_stamp) {
          write_ser_msg<Trajectory>(previous_traj, traj_file);
        }
        // Write the previous kinematic state if the current one is newer than the first command
        if (current_kine.time_stamp > previous_cmd.time_stamp) {
          write_ser_msg<Odometry>(previous_kine, kine_file);
        }
        // Write the rest of the messages that have been read.
        write_ser_msg<Trajectory>(current_traj, traj_file);
        write_ser_msg<Odometry>(current_kine, kine_file);
        write_ser_msg<AckermannControlCommand>(previous_cmd, cmd_file);
        write_ser_msg<AckermannControlCommand>(current_cmd, cmd_file);
        break;
      }
    }
  }

  // Write following messages to file
  while (reader.has_next()) {
    auto ser_msg = reader.read_next();
    if (ser_msg->topic_name == traj_topic) {
      write_ser_msg<Trajectory>(*ser_msg, traj_file);
    } else if (ser_msg->topic_name == kine_topic) {
      write_ser_msg<Odometry>(*ser_msg, kine_file);
    } else if (ser_msg->topic_name == cmd_topic) {
      write_ser_msg<AckermannControlCommand>(*ser_msg, cmd_file);
    }
  }

  return 0;
}

/// @brief Write file header and setup stream formatting flags.
/// @param file File to setup
void file_setup(std::ofstream & file)
{
  file << std::endl;
  file << "# SPDX-License-Identifier: Apache-2.0" << std::endl;
  file << "# Auto-generated from actuation_format_bag" << std::endl;
  file << std::hexfloat;
}
}  // namespace

int main(int argc, char const * argv[])
{
  if (argc != 2) {
    std::fprintf(stderr, "Expected 1 argument: rosbag path\n");
    return 1;
  }

  const char * rosbag_path = argv[1];

  std::ofstream traj_file("Trajectory.csv");
  if (traj_file.fail()) {
    std::fprintf(stderr, "Can't open Trajectory.csv\n");
    return 1;
  }
  file_setup(traj_file);

  std::ofstream kine_file("KinematicState.csv");
  if (kine_file.fail()) {
    std::fprintf(stderr, "Can't open KinematicState.csv\n");
    traj_file.close();
    return 1;
  }
  file_setup(kine_file);

  std::ofstream cmd_file("ControlCommand.csv");
  if (cmd_file.fail()) {
    std::fprintf(stderr, "Can't open ControlCommand.csv\n");
    traj_file.close();
    kine_file.close();
    return 1;
  }
  file_setup(cmd_file);

  int ret = write(rosbag_path, traj_file, kine_file, cmd_file);

  traj_file.close();
  kine_file.close();
  cmd_file.close();

  return ret;
}
