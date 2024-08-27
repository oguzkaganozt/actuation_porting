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

#include <actuation_player/actuation_player.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

static_assert(sizeof(float) == 4, "Unexpected float size");
static_assert(sizeof(double) == 8, "Unexpected double size");

namespace
{
/// @brief Maximum size for a stream.
constexpr auto max_size = std::numeric_limits<std::streamsize>::max();
/// @brief Maximum number of points in a trajectory.
constexpr auto traj_capacity = autoware_auto_planning_msgs_msg_Trajectory_Constants_CAPACITY;
/// @brief Frequency at which the kinematic state messages were recorded.
constexpr int recorded_frequency = 40;

/// @brief First timestamp from the recorded messages.
uint64_t recording_start_ = 0;
/// @brief System time at which the replay started.
uint64_t replay_start_ = 0;
/// @brief Whether the replay runs in a loop or not.
bool run_loop_ = false;

/// @brief Structured data to hold the trajectory information from the recorded file.
struct traj_data
{
  uint64_t timestamp_ns;
  std::array<TrajectoryPoint, traj_capacity> traj_buffer;
  Trajectory msg;
};

/// @brief Structured data to hold the kinematic state information from the recorded file.
struct kine_data
{
  uint64_t timestamp_ns;
  KinematicState msg;
};

/// @brief Create a DDS writer.
/// @param participant Participant DDS entity
/// @param desc Descriptor of the message type to be written
/// @param name Name of the topic to write to
/// @param qos Pointer to the QoS
/// @return DDS entity of the created writer or error code
dds_entity_t create_writer(
  dds_entity_t participant,
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos)
{
  dds_entity_t topic = dds_create_topic(participant, desc, name, NULL, NULL);
  if (topic < 0) {
    std::fprintf(stderr, "dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
    return topic;
  }

  dds_entity_t writer = dds_create_writer(participant, topic, qos, NULL);
  if (writer < 0) {
    std::fprintf(stderr, "dds_create_writer (%s): %s\n", name, dds_strretcode(-writer));
    return writer;
  }

  // In case the replay isn't looped, wait for synchronization with a reader before continuing.
  if (!run_loop_) {
    uint32_t status = 0;
    while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
      dds_return_t rc = dds_get_status_changes(writer, &status);
      if (rc != DDS_RETCODE_OK) {
        std::fprintf(stderr, "dds_get_status_changes: %s\n", dds_strretcode(-rc));
        return rc;
      }

      /* Polling sleep. */
      dds_sleepfor(DDS_MSECS(20));
    }
  }

  return writer;
}

/// @brief Setup the DDS environment.
/// @param[out] traj_writer Trajectory writer
/// @param[out] kine_writer Kinematic State writer
/// @return Error code
int setup_dds(dds_entity_t & traj_writer, dds_entity_t & kine_writer)
{
  // Create a Participant for all DDS usage.
  dds_entity_t participant = dds_create_participant(DDS_DOMAIN_ACTUATION, NULL, NULL);
  if (participant < 0) {
    std::fprintf(stderr, "dds_create_participant: %s\n", dds_strretcode(-participant));
    return 1;
  }

  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  traj_writer = create_writer(
    participant,
    &autoware_auto_planning_msgs_msg_Trajectory_desc,
    "trajectory",
    qos
  );
  if (traj_writer < 0) {
    return 1;
  }

  kine_writer = create_writer(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleKinematicState_desc,
    "current_pose",
    qos
  );
  if (kine_writer < 0) {
    return 1;
  }

  return 0;
}

/// @brief Reset file and skip header.
/// @param[in,out] file File to setup
void file_setup(std::ifstream & file)
{
  // Reset file
  file.clear();
  file.seekg(0, std::ios_base::beg);

  // Skip header
  file.ignore(max_size, '\n');
  file.ignore(max_size, '\n');
  file.ignore(max_size, '\n');
}

/// @brief Extract next value from a comma-separated string.
/// @param[in,out] line String to extract from
/// @param[out] value Extracted value
void get_value(std::stringstream & line, float & value)
{
  std::string value_str;
  std::getline(line, value_str, ',');
  value = std::stof(value_str);
}

/// @brief Extract next value from a comma-separated string.
/// @param[in,out] line String to extract from
/// @param[out] value Extracted value
void get_value(std::stringstream & line, double & value)
{
  std::string value_str;
  std::getline(line, value_str, ',');
  value = std::stod(value_str);
}

/// @brief Extract next value from a comma-separated string.
/// @param[in,out] line String to extract from
/// @param[out] value Extracted value
void get_value(std::stringstream & line, uint32_t & value)
{
  std::string value_str;
  std::getline(line, value_str, ',');
  value = std::stoul(value_str);
}

/// @brief Extract next value from a comma-separated string.
/// @param[in,out] line String to extract from
/// @param[out] value Extracted value
void get_value(std::stringstream & line, uint64_t & value)
{
  std::string value_str;
  std::getline(line, value_str, ',');
  value = std::stoull(value_str);
}

/// @brief Extract a trajectory point from a comma-separated string.
/// @param[in,out] line String to extract from
/// @param[out] point Extracted trajectory point
void get_point(std::stringstream & line, TrajectoryPoint & point)
{
  get_value(line, point.pose.position.x);
  get_value(line, point.pose.position.y);
  get_value(line, point.pose.position.z);
  get_value(line, point.pose.orientation.x);
  get_value(line, point.pose.orientation.y);
  get_value(line, point.pose.orientation.z);
  get_value(line, point.pose.orientation.w);
  get_value(line, point.longitudinal_velocity_mps);
  get_value(line, point.lateral_velocity_mps);
  get_value(line, point.acceleration_mps2);
  get_value(line, point.heading_rate_rps);
  get_value(line, point.front_wheel_angle_rad);
  get_value(line, point.rear_wheel_angle_rad);
}

/// @brief Extract a line from a file.
/// @param[in,out] file File to extract from
/// @param[out] line_str Extracted line
/// @return Error code
int get_line(std::ifstream & file, std::string & line_str)
{
  std::getline(file, line_str);

  if (file.eof()) {
    return 1;
  } else if (file.fail()) {
    throw std::runtime_error{"Failed reading line"};
  }

  return 0;
}

/// @brief Extract next trajectory data from a file.
/// @param[in,out] file File to extract from
/// @param[out] data Extracted trajectory data
/// @return Error code
int get_data(std::ifstream & file, struct traj_data & data)
{
  std::string line_str {};
  if (get_line(file, line_str)) {
    return 1;
  }
  std::stringstream line {line_str};

  get_value(line, data.timestamp_ns);
  get_value(line, data.msg.points._length);
  if (data.msg.points._length > data.msg.points._maximum) {
    throw std::runtime_error{"Trajectory points overflow"};
  }

  for (size_t i = 0; i < data.msg.points._length; i++) {
    auto & time_from_start = data.msg.points._buffer[i].time_from_start;
    time_from_start.sec = (i + 1) / recorded_frequency;
    time_from_start.nanosec = ((i + 1) % recorded_frequency) * (1000000000 / recorded_frequency);
    get_point(line, data.msg.points._buffer[i]);
  }

  return 0;
}

/// @brief Extract next kinematic state data from a file.
/// @param[in,out] file File to extract from
/// @param[out] data Extracted kinematic state data
/// @return Error code
int get_data(std::ifstream & file, struct kine_data & data)
{
  std::string line_str {};
  if (get_line(file, line_str)) {
    return 1;
  }
  std::stringstream line {line_str};

  get_value(line, data.timestamp_ns);
  get_point(line, data.msg.state);

  return 0;
}

/// @brief Sleep until the next replay time.
/// @param timestamp_ns Timestamp of the next recorded message to play
/// @param replay_divider Divider of the replay speed
void sleep_until_replay(uint64_t timestamp_ns, float replay_divider)
{
  uint64_t awake_time = replay_start_ + replay_divider * (timestamp_ns - recording_start_);
  auto awake_time_chrono = std::chrono::nanoseconds(awake_time);
  std::this_thread::sleep_until(std::chrono::steady_clock::time_point(awake_time_chrono));
}

/// @brief Write virtual system time into the time message. It will differ from the actual system
///        time when replay_divider is not 1.
/// @param timestamp_ns Timestamp of the associated message
/// @param[out] stamp Time message to fill
void fill_stamp(uint64_t timestamp_ns, builtin_interfaces_msg_Time & stamp)
{
  uint64_t virtual_time = replay_start_ + (timestamp_ns - recording_start_);
  stamp.sec = virtual_time / 1000000000;
  stamp.nanosec = virtual_time % 1000000000;
}

/// @brief Write the current trajectory as a DDS message and extract the next one from file.
/// @param writer DDS writer
/// @param[in,out] file File to read from
/// @param[in,out] data Data buffer
/// @return Error code
int process_data(dds_entity_t writer, std::ifstream & file, traj_data & data)
{
  fill_stamp(data.timestamp_ns, data.msg.header.stamp);
  dds_write(writer, &data.msg);
  return get_data(file, data);
}

/// @brief Write the current kinematic state as a DDS message and extract the next one from file.
/// @param writer DDS writer
/// @param[in,out] file File to read from
/// @param[in,out] data Data buffer
/// @return Error code
int process_data(dds_entity_t writer, std::ifstream & file, kine_data & data)
{
  fill_stamp(data.timestamp_ns, data.msg.header.stamp);
  data.msg.state.time_from_start.sec = data.msg.header.stamp.sec;
  data.msg.state.time_from_start.nanosec = data.msg.header.stamp.nanosec;
  dds_write(writer, &data.msg);
  return get_data(file, data);
}

/// @brief Open files containing the recordings.
/// @param path_prefix Path to the directory containing the recordings
/// @param[in,out] traj_file Trajectory file
/// @param[in,out] kine_file Kinematic state file
/// @return Error code
int open_files(const char * path_prefix, std::ifstream & traj_file, std::ifstream & kine_file)
{
  std::string traj_path {path_prefix};
  traj_path.append("/Trajectory.csv");
  traj_file.open(traj_path);
  if (traj_file.fail()) {
    std::fprintf(stderr, "Can't open %s\n", traj_path.data());
    return 1;
  }

  std::string kine_path {path_prefix};
  kine_path.append("/KinematicState.csv");
  kine_file.open(kine_path);
  if (kine_file.fail()) {
    std::fprintf(stderr, "Can't open %s\n", kine_path.data());
    traj_file.close();
    return 1;
  }

  return 0;
}
}  // namespace

int actuation_player(
  const char * path_prefix, bool run_loop, float replay_divider, volatile bool * cancel)
{
  if (replay_divider <= 0) {
    std::fprintf(stderr, "Invalid divider\n");
    return 1;
  }
  run_loop_ = run_loop;

  std::ifstream traj_file {};
  std::ifstream kine_file {};
  if (open_files(path_prefix, traj_file, kine_file)) {
    return 1;
  }

  // Setup DDS
  dds_entity_t traj_writer {};
  dds_entity_t kine_writer {};
  if (!run_loop) {
    std::printf("Waiting for readers...\n");
  }
  if (setup_dds(traj_writer, kine_writer)) {
    traj_file.close();
    kine_file.close();
    return 1;
  }
  if (run_loop) {
    std::printf("Starting replay... Ctrl-C to cancel\n");
  } else {
    std::printf("...Done. Starting replay.\n");
  }

  // Initialize message structures
  char frame_id[] = "odom";
  struct traj_data traj {};
  traj.msg.points._maximum = traj.traj_buffer.size();
  traj.msg.points._buffer = traj.traj_buffer.data();
  traj.msg.header.frame_id = frame_id;
  struct kine_data kine {};
  kine.msg.header.frame_id = frame_id;

  int ret = 0;
  do {
    file_setup(traj_file);
    file_setup(kine_file);

    try {
      int traj_state = get_data(traj_file, traj);
      int kine_state = get_data(kine_file, kine);

      // Initialize timing information
      recording_start_ = std::min(traj.timestamp_ns, kine.timestamp_ns);
      auto now = std::chrono::steady_clock::now().time_since_epoch();
      replay_start_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

      // Publish messages in the recorded order
      while (!traj_state || !kine_state) {
        if (kine_state || (!traj_state && traj.timestamp_ns < kine.timestamp_ns)) {
          sleep_until_replay(traj.timestamp_ns, replay_divider);
          traj_state = process_data(traj_writer, traj_file, traj);
        } else {
          sleep_until_replay(kine.timestamp_ns, replay_divider);
          kine_state = process_data(kine_writer, kine_file, kine);
        }
      }
    } catch (const std::runtime_error & e) {
      std::fprintf(stderr, "Error: %s\n", e.what());
      ret = 1;
      break;
    }
  } while (run_loop && !(cancel && *cancel));

  traj_file.close();
  kine_file.close();

  return ret;
}
