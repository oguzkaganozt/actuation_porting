# Autoware MPC Lateral Controller Safety Island

## Components

| Component | Version |
|--------------|---------------|
| Zephyr RTOS  | [3.6.0](https://github.com/zephyrproject-rtos/zephyr/commit/6aeb7a2b96c2b212a34f00c0ad3862ac19e826e8) |
| CycloneDDS  | [0.11.x](https://github.com/eclipse-cyclonedds/cyclonedds/commit/7c253ad3c4461b10dc4cac36a257b097802cd043) |
| Autoware    | [2025.02](https://github.com/autowarefoundation/autoware/tree/2025.02) |
| Autoware.Universe | [0.40.0](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0) |
| Autoware.msgs | [1.3.0](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0) |

## Porting Status

### Completed ✅
- autoware_msgs
- autoware_osqp_interface
- autoware_universe_utils_min
- autoware_motion_utils_min
- autoware_interpolation_min
- autoware_vehicle_info_utils

### Pending 🔄
- autoware_trajectory_follower_base
- autoware_mpc_lateral_controller

## ROS to Zephyr Migration Tasks

| ROS Component | Zephyr Target | Status |
|--------------|---------------|---------|
| Node Launch  | POSIX Threads | ✅ Completed |
| Parameters   | Configuration File Reader | ✅ Completed |
| RCL Logging  | Zephyr Logger | ✅ Completed |
| RCL Timers   | Zephyr Timers | ⏳ Pending |
| RCL Callbacks| Zephyr Callbacks | ⏳ Pending |

### Key Notes

- Validate launch structure and synchronization with the native ROS2 nodes
- Validate the message conversion between ROS2 and Zephyr
- Validate cycloneDDS <-> ROS2 communication with the native ROS2 nodes
- Validate the parameter handling with the native ROS2 nodes
- Validate the logging and logging structure with the native ROS2 nodes
- Validate the timer and callback structure with the native ROS2 nodes
