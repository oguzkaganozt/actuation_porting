# Autoware MPC Lateral Controller Porting

## Autoware Components & Versions

| Component          | Version | Repository Link |
|-------------------|---------|-----------------|
| Autoware          | 2025.02 | [Link](https://github.com/autowarefoundation/autoware/tree/2025.02) |
| Autoware.Universe | 0.40.0  | [Link](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0) |
| Autoware.msgs     | 1.3.0   | [Link](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0) |

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
| Node Launch  | Zephyr Threads | ✅ Completed |
| Parameters   | Zephyr Parameters | ✅ Completed |
| RCL Logging  | Zephyr Logger | ✅ Completed |
| RCL Timers   | Zephyr Timers | ⏳ Pending |
| RCL Callbacks| Zephyr Callbacks | ⏳ Pending |
