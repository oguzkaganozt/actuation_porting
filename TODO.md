# TODO List (as of 2024-10-30)

## Development Environment
- [X] Fix devcontainer

## Porting and Migration
- [X] Complete porting **TF2** package
- [+] Switch **threading** mechanism to pthread instead of rcl thread of ROS2 (PARTIAL PORTING DONE)
- [X] Use Zephyr **logging** mechanism instead of plain printf or ROS logger
- [+] **Actuation Nodes to be ported**
    - [X] autoware_cmake 
    - [X] autoware_helper_utils
    - [+] autoware_motion_utils (PARTIAL PORTING DONE)
    - [X] autoware_vehicle_info_utils
    - [+] interpolation (PARTIAL PORTING DONE)
    - [+] osqp_interface (PARTIAL PORTING DONE)
    - [+] autoware_trajectory_follower_base (PARTIAL PORTING DONE)
    - [ ] autoware_mpc_lateral_controller
    - [ ] autoware_universe_utils (NEEDS TO BE INVESTIGATED FOR SELECTIVE PORTING: autoware_helper_utils can be used instead)

## Message Handling
- [ ] Implement **message conversion** or compilation into idlc
- [ ] Introduce **message queue** within the actuation service (OPTIONAL)

## Launch Files
- [ ] Validate launching synchronization of the actuation service

## Configuration Management
- [ ] Create **configuration** file reader to replace inline configuration (OPTIONAL)

## Testing
- [ ] Develop **unit tests** for the actuation service
- [ ] Create **integration tests** for the actuation service (OPTIONAL)
