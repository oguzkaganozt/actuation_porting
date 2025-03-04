# Autoware Trajectory Follower - Safety Island

## Components

| Component | Version |
|--------------|---------------|
| Zephyr RTOS  | [3.6.0](https://github.com/zephyrproject-rtos/zephyr/commit/6aeb7a2b96c2b212a34f00c0ad3862ac19e826e8) |
| CycloneDDS  | [0.11.x](https://github.com/eclipse-cyclonedds/cyclonedds/commit/7c253ad3c4461b10dc4cac36a257b097802cd043) |
| Autoware    | [2025.02](https://github.com/autowarefoundation/autoware/tree/2025.02) |
| Autoware.Universe | [0.40.0](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0) |
| Autoware.msgs | [1.3.0](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0) |

## Autoware Nodes
| Node | Input | Output | Status |
|------|--------------------|-------------------|--------|
| **vehicle_info** | None | None | ✅ Completed |
| **autoware_mpc_lateral_controller** |**Input:**`autoware_planning_msgs/Trajectory`<br>**Description:** Reference trajectory to follow<hr>**Input:** `nav_msgs/Odometry`<br>**Description:** Current odometry<hr>**Input:** `autoware_vehicle_msgs/SteeringReport`<br>**Description:** Current steering |**Output:**`autoware_control_msgs/Lateral`<br>**Description:** Lateral control command<hr>**Output:**`LateralSyncData::steer angle convergence`<br>**Description:** Steer angle convergence<hr>**Topic:** `lateral/predicted_trajectory`<br>**Message:** `autoware_planning_msgs/Trajectory`<br>**Description:** (The trajectory size will be empty when the controller is in an emergency such as too large deviation from the planning trajectory) | ⏳ Pending |
| **autoware_pid_longitudinal_controller** |**Input** `autoware_planning_msgs/Trajectory`<br>**Description:** Reference trajectory to follow<hr>**Input** `nav_msgs/Odometry`<br>**Description:** Current odometry|**Output:** `autoware_control_msgs/Longitudinal`<br>**Description:** Longitudinal control command | ⏳ Pending |
| **autoware_trajectory_follower_node** |**Topic:** `/planning/scenario_planning/trajectory` <br>**Message:** `autoware_planning_msgs/Trajectory`<br>**Description:** Reference trajectory to follow<hr>**Topic:** `/localization/kinematic_state`<br>**Message:** `nav_msgs/Odometry`<br>**Description:** Current odometry<hr>**Topic:**  CHECK! <br>**Message:**`autoware_vehicle_msgs/SteeringReport`<br>**Description:** Current steering |**Topic:** `/vehicle/command/manual_control_cmd` <br>**Message:** `autoware_control_msgs/Control`<br> **Description:** Message containing both lateral and longitudinal commands.<br> | ⏳ Pending |


## Autoware Node Dependencies

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
| autoware_pid_longitudinal_controller | ⏳ Pending |
| autoware_trajectory_follower_node | ⏳ Pending |

## ROS Utils Migration

| ROS Component | Zephyr Target | Status |
|--------------|---------------|---------|
| RCL Logging  | Zephyr Logger | ✅ Completed |
| RCL Node     | Zephyr k_threads | ✅ Completed |
| RCL Timers   | Zephyr Timers | ✅ Completed |
| RCL Publisher | CycloneDDS | ✅ Completed |
| RCL Subscriber | CycloneDDS | ✅ Completed |
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
