
# Add source files
list(APPEND APP_SOURCES
  src/autoware_motion_utils_min/src/trajectory/interpolation.cpp
  src/autoware_motion_utils_min/src/trajectory/trajectory.cpp
  src/autoware_motion_utils_min/src/vehicle/vehicle_state_checker.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/autoware_motion_utils_min/include
)
