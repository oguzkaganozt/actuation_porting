
# Add source files
list(APPEND APP_SOURCES
  src/nodes/autoware_interpolation_min/src/linear_interpolation.cpp
  src/nodes/autoware_interpolation_min/src/spline_interpolation.cpp
  src/nodes/autoware_interpolation_min/src/spline_interpolation_points_2d.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/nodes/autoware_interpolation_min/include
)
