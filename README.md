<!--
# Copyright (c) 2024-2025, Arm Limited.
#
# SPDX-License-Identifier: Apache-2.0
-->

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
| Node | Subscribed Topics | Published Topics | Status |
|------|--------------------|-------------------|--------|
| **vehicle_info** | None | None | ✅ Completed |
| **autoware_pid_longitudinal_controller** | None | **Topic:** `longitudinal/diagnostic` <br>**Message:** `tier4_debug_msgs/Float32MultiArrayStamped`<br> **Description:** Longitidunal diagnostics. <hr>**Topic:** `longitudinal/slope_angle` <br>**Message:** `tier4_debug_msgs/Float32MultiArrayStamped`<br> **Description:** Longitidunal slope angle<hr>**Topic:** `longitudinal/stop_reason` <br>**Message:** `visualization_msgs/Marker`<br> **Description:** Stop reason<hr> | ⏳ Pending |
| **autoware_mpc_lateral_controller** | None |**Topic:** `lateral/diagnostic` <br>**Message:** `tier4_debug_msgs/Float32MultiArrayStamped`<br> **Description:** Lateral diagnostics <hr>**Topic:** `lateral/predicted_trajectory`<br>**Message:** `autoware_planning_msgs/Trajectory`<br>**Description:** (The trajectory size will be empty <br>when the controller is in an emergency such as,<br> too large deviation from the planning trajectory) | ⏳ Pending |
| **autoware_trajectory_follower_node** |**Topic:** `/planning/scenario_planning/trajectory` <br>**Message:** `autoware_planning_msgs/Trajectory`<br>**Description:** Reference trajectory to follow<hr>**Topic:** `/localization/kinematic_state`<br>**Message:** `nav_msgs/Odometry`<br>**Description:** Current odometry<hr>**Topic:** `/vehicle/status/steering_status` <br>**Message:**`autoware_vehicle_msgs/SteeringReport`<br>**Description:** Current steering <hr>**Topic:** `/localization/acceleration` <br>**Message:**`geometry_msgs/AccelWithCovarianceStamped`<br>**Description:** Current acceleration<hr>**Topic:** `/system/operation_mode/state` <br>**Message:**`autoware_adapi_v1_msgs/OperationModeState`<br>**Description:** System operation mode |**Topic:** `control_cmd` <br>**Message:** `autoware_control_msgs/Control`<br> **Description:** Message containing both lateral and longitudinal commands.| ⏳ Pending |

![trajectory_follower](https://github.com/user-attachments/assets/327f9b8e-e089-4d9b-ada7-621fbcb20e97)

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
| autoware_mpc_lateral_controller | ✅v Completed |
| autoware_pid_longitudinal_controller | ✅ Completed |
| autoware_trajectory_follower_node | ✅ Completed |

## ROS Utils Migration

| ROS Component | Zephyr Target | Status |
|--------------|---------------|---------|
| RCL Logging  | Zephyr Logger | ✅ Completed |
| RCL Node     | Zephyr k_threads | ✅ Completed |
| RCL Timers   | Zephyr Timers | ✅ Completed |
| RCL Publisher | CycloneDDS | ✅ Completed |
| RCL Subscriber | CycloneDDS | ✅ Completed |
