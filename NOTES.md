# Autoware MPC Lateral Controller Porting

## Autoware Version

- Autoware: [2025.02](https://github.com/autowarefoundation/autoware/tree/2025.02)
- Autoware.Universe: [0.40.0](https://github.com/autowarefoundation/autoware.universe/tree/0.40.0)
- Autoware.msgs: [1.3.0](https://github.com/autowarefoundation/autoware_msgs/tree/1.3.0)

## PORTING: Autoware Universe Utils

### Geometry

- [ ] geometry/geometry.hpp
  - [ ] sub-dependencies...
- [ ] geometry/boost_geometry.hpp
  - [ ] sub-dependencies...
- [ ] geometry/boost_polygon_utils.hpp
  - [ ] sub-dependencies...
- [ ] geometry/gjk_2d.hpp
  - [ ] sub-dependencies...
- [ ] geometry/pose_deviation.hpp
  - [ ] sub-dependencies...

### Math

- [ ] math/constants.hpp
  - [ ] sub-dependencies...
- [ ] math/normalization.hpp
  - [ ] sub-dependencies...
- [ ] math/unit_conversion.hpp
  - [ ] sub-dependencies...

### ROS

- [ ] ros/marker_helper.hpp
  - [ ] sub-dependencies...

## PORTING: Autoware Motion Utils

### Trajectory

- [ ] motion_utils/trajectory/trajectory.hpp
  - [ ] sub-dependencies...

## PORTING: Autoware Interpolation

- [ ] interpolation/linear_interpolation.hpp
  - [ ] sub-dependencies...
- [ ] interpolation/spline_interpolation.hpp
  - [ ] sub-dependencies...
- [ ] interpolation/zero_order_hold.hpp
  - [ ] sub-dependencies...

## PORTING: Autoware Trajectory Follower Base

- [ ] trajectory_follower_base/lateral_controller_base.hpp
  - [ ] sub-dependencies...
- [ ] trajectory_follower_base/control_horizon.hpp
  - [ ] sub-dependencies...
