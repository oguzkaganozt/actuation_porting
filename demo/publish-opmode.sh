#!/bin/bash

source /opt/ros/humble/setup.bash
source /opt/autoware/setup.bash

ros2 topic pub /system/operation_mode/state \
  autoware_adapi_v1_msgs/msg/OperationModeState \
  "{stamp: {sec: 1750169505, nanosec: 298781434},
    mode: 1,
    is_autoware_control_enabled: true,
    is_in_transition: false,
    is_stop_mode_available: true,
    is_autonomous_mode_available: true,
    is_local_mode_available: true,
    is_remote_mode_available: true}" \
  -1
