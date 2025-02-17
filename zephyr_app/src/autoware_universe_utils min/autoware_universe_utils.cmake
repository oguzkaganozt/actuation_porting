macro(add_autoware_universe_utils)
  # Find dependencies
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED)
  
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_universe_utils/src/geometry/geometry.cpp
    src/autoware_universe_utils/src/geometry/boost_polygon_utils.cpp
    src/autoware_universe_utils/src/geometry/gjk_2d.cpp
    src/autoware_universe_utils/src/geometry/pose_deviation.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_universe_utils/include
    "${EIGEN3_INCLUDE_DIR}"
  )
endmacro()
