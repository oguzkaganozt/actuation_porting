
# Add source files
list(APPEND APP_SOURCES
  src/autoware/autoware_interpolation/src/linear_interpolation.cpp
  src/autoware/autoware_interpolation/src/spline_interpolation.cpp
  src/autoware/autoware_interpolation/src/spline_interpolation_points_2d.cpp
  src/autoware/autoware_interpolation/src/spherical_linear_interpolation.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/autoware/autoware_interpolation/include
)
