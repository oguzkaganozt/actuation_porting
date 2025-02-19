macro(add_autoware_universe_utils)
  # Find dependencies
  find_package(Eigen3 REQUIRED)
  
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_universe_utils_min/src/geometry/geometry.cpp
    src/autoware_universe_utils_min/src/geometry/alt_geometry.cpp
    src/autoware_universe_utils_min/src/geometry/pose_deviation.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_universe_utils_min/include
    "${EIGEN3_INCLUDE_DIR}"
  )
endmacro()
