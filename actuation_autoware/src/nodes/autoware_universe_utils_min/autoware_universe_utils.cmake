
# Add source files
list(APPEND APP_SOURCES
  src/nodes/autoware_universe_utils_min/src/geometry/geometry.cpp
  src/nodes/autoware_universe_utils_min/src/geometry/alt_geometry.cpp
  src/nodes/autoware_universe_utils_min/src/geometry/pose_deviation.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/nodes/autoware_universe_utils_min/include
  "${EIGEN3_INCLUDE_DIR}"
)
