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
