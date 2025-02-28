# Autoware MPC Lateral Controller Safety Island

## Components

| Component | Version |
|--------------|---------------|
| Zephyr RTOS  | [3.6.0](https://github.com/zephyrproject-rtos/zephyr/commit/6aeb7a2b96c2b212a34f00c0ad3862ac19e826e8) |
| CycloneDDS  | [0.11.x](https://github.com/eclipse-cyclonedds/cyclonedds/commit/7c253ad3c4461b10dc4cac36a257b097802cd043) |
| Autoware    | [2025.02](https://github.com/autowarefoundation/autoware/tree/2025.02) |
| Autoware.Universe | [0.40.0](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0) |
| Autoware.msgs | [1.3.0](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0) |

## Autoware Components Porting

| Component | Status |
|-----------|---------|
| autoware_msgs | ✅ Completed |
| autoware_osqp_interface | ✅ Completed |
| autoware_universe_utils | ✅ Completed |
| autoware_motion_utils | ✅ Completed |
| autoware_interpolation | ✅ Completed |
| autoware_vehicle_info_utils | ✅ Completed |
| autoware_trajectory_follower_base | ✅ Completed |
| autoware_mpc_lateral_controller | ⏳ Pending |

## ROS Utils Migration

| ROS Component | Zephyr Target | Status |
|--------------|---------------|---------|
| RCL Logging  | Zephyr Logger | ✅ Completed |
| RCL Node     | POSIX Threads vs k_threads | ⏳ Pending |
| RCL Parameters | Custom File Reader | ⏳ Pending |
| RCL Timers   | Zephyr Timers | ⏳ Pending |
| RCL Subscribers | CycloneDDS | ⏳ Pending |
| RCL Publishers | CycloneDDS | ⏳ Pending |
| RCL Callbacks| Zephyr Callbacks | ⏳ Pending |

## Node Creation/Launch

| Node | Status |
|------|--------|
| vehicle_info | ⏳ Pending |
| vehicle_state_checker | ⏳ Pending |
| autoware_mpc_lateral_controller | ⏳ Pending |

### Development Notes

- Validate node functionality with native ROS2 nodes
- change memory configuration of the cyclonedds with zephyr allocator -> https://cyclonedds.io/docs/cyclonedds/latest/config/allocation-config.html
- Validate launch structure and synchronization with the native ROS2 nodes
- Validate the message conversion between ROS2 and Zephyr
- Validate cycloneDDS <-> ROS2 communication with the native ROS2 nodes
- Validate the parameter handling with the native ROS2 nodes
- Validate the logging and logging structure with the native ROS2 nodes
- Validate the timer and callback structure with the native ROS2 nodes
