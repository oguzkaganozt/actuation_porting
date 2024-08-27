<!--
# Copyright (c) 2023-2024, Arm Limited.
#
# SPDX-License-Identifier: Apache-2.0
-->

# actuation_format_bag

## Package

This package follows the ROS2 package design. It can be built and tested by
using
[colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

## Design

### Usage

While running the Autoware Actuation demo, record the topics of interest with:

``` sh
ros2 bag record -o record_out /localization/kinematic_state /planning/scenario_planning/trajectory /control/trajectory_follower/control_cmd
```

This package can be used to read the resulting rosbag and write the relevant
data to `KinematicState.csv`, `Trajectory.csv` and `ControlCommand.csv` in a CSV
format.

Usage:

```sh
./actuation_format_bag rosbag_path
```

### Format

The first 3 lines of the files are for the header and should be skipped when
reading them back.

Given the following structures:

```cpp
struct TrajectoryPoint {
  uint64_t timestamp_ns;
  double position_x;
  double position_y;
  double position_z;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  float longitudinal_velocity_mps;
  float lateral_velocity_mps;
  float acceleration_mps2;
  float heading_rate_rps;
  float front_wheel_angle_rad;
  float rear_wheel_angle_rad;
};

struct Trajectory {
  uint64_t timestamp_ns;
  uint32_t size;
  struct TrajectoryPoint[size];
};

struct VehicleControlCommand {
  uint64_t timestamp_ns;
  float long_accel_mps2;
  float velocity_mps;
  float front_wheel_angle_rad;
  float rear_wheel_angle_rad;
};
```

`KinematicState.csv` contains a textual representation of `struct
TrajectoryPoint`; `Trajectory.csv` contains a textual representation of `struct
Trajectory`; `ControlCommand.csv` contains a textual representation of `struct
VehicleControlCommand`.

`float` and `double` values are written with an hexadecimal format.

The first repeating commands are removed in order to get a unique first command.
