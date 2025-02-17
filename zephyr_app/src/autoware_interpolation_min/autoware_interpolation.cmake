macro(add_autoware_interpolation)
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_interpolation/src/linear_interpolation.cpp
    src/autoware_interpolation/src/spline_interpolation.cpp
    src/autoware_interpolation/src/zero_order_hold.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_interpolation/include
  )
endmacro()
