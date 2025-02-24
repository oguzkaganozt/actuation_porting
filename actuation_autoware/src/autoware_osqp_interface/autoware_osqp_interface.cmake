
# Add source files
list(APPEND APP_SOURCES
  src/autoware_osqp_interface/src/csc_matrix_conv.cpp
  src/autoware_osqp_interface/src/osqp_interface.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/autoware_osqp_interface/include
  "${OSQP_INCLUDE_DIR}"
  "${EIGEN3_INCLUDE_DIR}"
)
