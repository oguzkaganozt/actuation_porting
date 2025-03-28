# OSQP
- removed throws use printfs instead
- Eigen: replaced fabsl with fabs

## UNIVERSE UTILS
- All:
    -- removed throws use printfs instead

- alt_geometry:
    -- We can also use eigen vector2d and  vector3d for point2d and point3d representation.  Now it is custom implementation.
    -- removed alt namespace as now we are using it for main geometry
    
- geometry:
    -- tf2::getYaw vs getYaw(custom)
    -- tf2::lerp vs lerp(custom)
    -- tf2::slerp vs slerp(custom)
    -- tf2::getRPY vs getRPY(custom)

## MPC LATERAL CONTROLLER
- generateDiagData removed for the simplicity
- Clock are now using chrono clock with Clock::now() instead of ros clock
- removed param callbacks as we are not using ros2
- Eigen: replaced fabsl with fabs
- Eliminate fmt::format
- Use wrap for wrapping the dds sequences prior manipulation
- replace warn_throttle and error_throttle with round_robin_logger
- removed publishDebugValues for the simplicity
- replaced rclcpp::ok() with while(1)
- some math constants not available in the zephyr toolchain, so added them to the universe_utils

## PID LONGITUDINAL CONTROLLER
- removed #include "autoware/motion_utils/trajectory/conversion.hpp"
- diag_updater_ removed as we are not using ros2
- removed param callbacks as we are not using ros2

