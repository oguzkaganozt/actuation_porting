# Autoware MPC Lateral Controller Safety Island

## Components

| Component | Version |
|--------------|---------------|
| Zephyr RTOS  | [3.6.0](https://github.com/zephyrproject-rtos/zephyr/commit/6aeb7a2b96c2b212a34f00c0ad3862ac19e826e8) |
| CycloneDDS  | [0.11.x](https://github.com/eclipse-cyclonedds/cyclonedds/commit/7c253ad3c4461b10dc4cac36a257b097802cd043) |
| Autoware    | [2025.02](https://github.com/autowarefoundation/autoware/tree/2025.02) |
| Autoware.Universe | [0.40.0](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0) |
| Autoware.msgs | [1.3.0](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0) |

## Autoware Nodes
| Node | Subscribed Topics | Published Topics | Status |
|------|--------------------|-------------------|--------|
| **vehicle_info** | None | None | ✅ Completed |
| **autoware_mpc_lateral_controller** | • `autoware_planning_msgs/Trajectory`<br>Reference trajectory to follow<br>• `nav_msgs/Odometry`<br>Current odometry<br>•`autoware_vehicle_msgs/SteeringReport`<br>Current steering | • `autoware_control_msgs/Lateral`<br>Lateral control command<br>• `LateralSyncData::steer angle convergence`<br>Steer angle convergence<br>• `autoware_planning_msgs::Trajectory`<br>Predicted trajectory.<br>(The trajectory size will be empty when the controller is in an emergency such as too large deviation from the planning trajectory) | ⏳ Pending |

## Autoware Dependencies

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
| RCL Node     | Zephyr k_threads | ✅ Completed |
| RCL Timers   | Zephyr Timers | ✅ Completed |
| RCL Publisher | CycloneDDS | ⏳ Pending |
| RCL Subscriber | CycloneDDS | ⏳ Pending |
| RCL Callback | Lambda | ⏳ Pending |

### Development Notes

- Validate node functionality with native ROS2 nodes
- change memory configuration of the cyclonedds with zephyr allocator -> https://cyclonedds.io/docs/cyclonedds/latest/config/allocation-config.html
- Validate launch structure and synchronization with the native ROS2 nodes
- Validate the message conversion between ROS2 and Zephyr
- Validate cycloneDDS <-> ROS2 communication with the native ROS2 nodes
- Validate the parameter handling with the native ROS2 nodes
- Validate the logging and logging structure with the native ROS2 nodes
- Validate the timer and callback structure with the native ROS2 nodes
