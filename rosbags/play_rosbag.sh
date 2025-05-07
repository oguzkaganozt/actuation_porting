#!/bin/bash

# shellcheck disable=SC1091
source "/opt/ros/humble/setup.bash"
source "/opt/autoware/setup.bash"

ros2 bag play --loop --start-offset 12 --rate 0.1 manual-planning-sim \
    --topics /vehicle/status/steering_status
    # /planning/scenario_planning/trajectory \
    # /localization/kinematic_state \
    # /localization/acceleration \
    # /system/operation_mode/state