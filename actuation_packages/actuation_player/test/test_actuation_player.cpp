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

#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <future>
#include <string>

#include <actuation_player/actuation_player.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr auto timeout = 100ms;

constexpr KinematicState state1 {
  {},
  {
    {},
    {
      {
        0x1.d7a8f9f43d12cp+11,
        0x1.1ffcb6c816b82p+16,
        0x1.3635a7ac1d449p+4
      },
      {
        0x0p+0,
        0x0p+0,
        0x1.b5498ab2cbbp-1,
        0x1.0a50b7c5341eap-1
      }
    },
    0x1.ed816cp+1,
    0x0p+0,
    0x0p+0,
    -0x1.d37b6ep-7,
    0x0p+0,
    0x0p+0
  },
  {}
};

constexpr KinematicState state2 {
  {},
  {
    {},
    {
      {
        0x1.d7a78fcbaa101p+11,
        0x1.1ffcccb41524dp+16,
        0x1.362ff3223e98dp+4
      },
      {
        0x0p+0,
        0x0p+0,
        0x1.b53d942f71a72p-1,
        0x1.0a645b654fca8p-1
      }
    },
    0x1.ecaceep+1,
    0x0p+0,
    0x0p+0,
    -0x1.c3fa4ap-7,
    0x0p+0,
    0x0p+0
  },
  {}
};

constexpr TrajectoryPoint trajectory_last {
  {},
  {
    {
      0x1.d6488db71171fp+11,
      0x1.200b0e63541d9p+16,
      0x1.34fd3246b2cc3p+4
    },
    {
      0x1.de8aead7367c6p-13,
      -0x1.2479cbdb466f7p-16,
      0x1.fe83281dd5973p-1,
      0x1.3803d749968ddp-4
    }
  },
  0x1.e31ffap+0,
  0x0p+0,
  -0x1.007efp-1,
  0x0p+0,
  0x1.6ea278p-2,
  0x0p+0
};

constexpr uint32_t trajectory_length {100};

std::atomic<bool> trajectory_received {false};
std::atomic<bool> state1_received {false};
std::atomic<bool> state2_received {false};

bool is_equal(const geometry_msgs_msg_Point & a, const geometry_msgs_msg_Point & b)
{
  return a.x == b.x &&
         a.y == b.y &&
         a.z == b.z;
}

bool is_equal(const geometry_msgs_msg_Quaternion & a, const geometry_msgs_msg_Quaternion & b)
{
  return a.x == b.x &&
         a.y == b.y &&
         a.z == b.z &&
         a.w == b.w;
}

bool is_equal(const geometry_msgs_msg_Pose & a, const geometry_msgs_msg_Pose & b)
{
  return is_equal(a.position, b.position) &&
         is_equal(a.orientation, b.orientation);
}

bool is_equal(const TrajectoryPoint & a, const TrajectoryPoint & b)
{
  return is_equal(a.pose, b.pose) &&
         a.longitudinal_velocity_mps == b.longitudinal_velocity_mps &&
         a.acceleration_mps2 == b.acceleration_mps2 &&
         a.heading_rate_rps == b.heading_rate_rps &&
         a.front_wheel_angle_rad == b.front_wheel_angle_rad;
}

bool is_equal(const KinematicState & a, const KinematicState & b)
{
  return is_equal(a.state, b.state);
}

/// @brief Take message from DDS reader
/// @param rd Reader
/// @param[out] sample Buffer into which data is read
/// @return Whether the operation was successful
bool get_msg(dds_entity_t rd, void * sample)
{
  dds_sample_info_t info;
  dds_return_t rc = dds_take(rd, &sample, &info, 1, 1);
  EXPECT_GE(rc, 0);
  EXPECT_TRUE(info.valid_data);
  if (rc > 0 && info.valid_data) {
    return true;
  }
  return false;
}

/// @brief Trajectory callback
/// @param rd Reader
/// @param arg Unused
void on_trajectory(dds_entity_t rd, void * arg)
{
  (void) arg;
  Trajectory msg {};
  if (get_msg(rd, reinterpret_cast<void *>(&msg))) {
    ASSERT_EQ(msg.points._length, trajectory_length);
    ASSERT_TRUE(is_equal(msg.points._buffer[trajectory_length - 1], trajectory_last));
    trajectory_received.store(true);
  }
}

/// @brief Kinematic State callback
/// @param rd Reader
/// @param arg Unused
void on_state(dds_entity_t rd, void * arg)
{
  (void) arg;
  KinematicState msg {};
  if (get_msg(rd, reinterpret_cast<void *>(&msg))) {
    if (is_equal(msg, state1)) {
      state1_received.store(true);
    } else if (is_equal(msg, state2)) {
      state2_received.store(true);
    } else {
      FAIL() << "Kinematic State message has incorrect values.";
    }
  }
}

/// @brief Verify return value from the function to test and verify that it resulted in receiving
///        the expected DDS messages.
/// @param path_prefix actuation_player argument
/// @param run_loop actuation_player argument
/// @param replay_divider actuation_player argument
void test_player_output(const char * path_prefix, bool run_loop, float replay_divider)
{
  trajectory_received.store(false);
  state1_received.store(false);
  state2_received.store(false);

  ASSERT_EQ(actuation_player(path_prefix, run_loop, replay_divider), 0);

  EXPECT_TRUE(trajectory_received.load());
  EXPECT_TRUE(state1_received.load());
  EXPECT_TRUE(state2_received.load());
}

/// @brief Get path to current executable.
/// @param[out] self_path Path to be filled
void get_self_path(std::string & self_path)
{
  char buf[256];
  ssize_t ret = readlink("/proc/self/exe", buf, sizeof(buf));
  ASSERT_NE(ret, -1);
  ASSERT_NE(ret, static_cast<ssize_t>(sizeof(buf)));
  buf[ret] = '\0';

  self_path = buf;
  size_t bin_dir = self_path.find_last_of('/');
  self_path.replace(bin_dir + 1, self_path.size() - (bin_dir + 1), "");
}
}  // namespace

TEST(MessageConverter, Sanity) {
  // Setup testbench DDS readers
  dds_entity_t participant = dds_create_participant(DDS_DOMAIN_ACTUATION, NULL, NULL);
  ASSERT_GE(participant, 0);
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  dds_entity_t trajectory_topic = dds_create_topic(
    participant,
    &autoware_auto_planning_msgs_msg_Trajectory_desc,
    "trajectory",
    NULL,
    NULL
  );
  ASSERT_GE(trajectory_topic, 0);
  dds_listener_t * trajectory_listener = dds_create_listener(NULL);
  dds_lset_data_available(trajectory_listener, on_trajectory);
  ASSERT_GE(dds_create_reader(participant, trajectory_topic, qos, trajectory_listener), 0);

  dds_entity_t state_topic = dds_create_topic(
    participant,
    &autoware_auto_vehicle_msgs_msg_VehicleKinematicState_desc,
    "current_pose",
    NULL,
    NULL
  );
  ASSERT_GE(state_topic, 0);
  dds_listener_t * state_listener = dds_create_listener(NULL);
  dds_lset_data_available(state_listener, on_state);
  ASSERT_GE(dds_create_reader(participant, state_topic, qos, state_listener), 0);

  std::string self_path;
  get_self_path(self_path);

  // Test sanity
  test_player_output(self_path.data(), false, 1);

  // Test path
  ASSERT_NE(actuation_player("", false, 1), 0);

  // Test divider
  ASSERT_NE(actuation_player(self_path.data(), false, 0), 0);
  ASSERT_NE(actuation_player(self_path.data(), false, -1), 0);
  test_player_output(self_path.data(), false, 10);
  test_player_output(self_path.data(), false, 0.1);

  // Test loop
  {
    trajectory_received.store(false);
    state1_received.store(false);
    state2_received.store(false);

    volatile bool cancel = false;
    auto a = std::async(std::launch::async, actuation_player, self_path.data(), true, 1, &cancel);

    std::this_thread::sleep_for(timeout);
    EXPECT_TRUE(trajectory_received.load());
    EXPECT_TRUE(state1_received.load());
    EXPECT_TRUE(state2_received.load());

    trajectory_received.store(false);
    state1_received.store(false);
    state2_received.store(false);
    std::this_thread::sleep_for(timeout);
    EXPECT_TRUE(trajectory_received.load());
    EXPECT_TRUE(state1_received.load());
    EXPECT_TRUE(state2_received.load());

    cancel = true;
    EXPECT_EQ(a.wait_for(timeout), std::future_status::ready);
  }
}
