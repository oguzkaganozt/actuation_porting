#!/bin/bash

# shellcheck disable=SC1091
source "/opt/ros/humble/setup.bash"
source "/opt/autoware/setup.bash"

ros2 bag play --start-offset 35 --rate 0.7 /actuation/actuation_module/test/rosbags/manual-planning-sim \
    --topics /vehicle/status/steering_status \
    /planning/scenario_planning/trajectory \
    /localization/kinematic_state \
    /localization/acceleration \
    /system/operation_mode/state
