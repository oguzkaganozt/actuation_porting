macro(add_osqp_interface SRCS INCLUDE_DIRS)
  # Find dependencies
  find_package(Eigen3 REQUIRED)
  find_package(osqp REQUIRED)
  get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
  get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

  # Add source files
  list(APPEND ${SRCS}
    src/osqp_interface/src/csc_matrix_conv.cpp
    src/osqp_interface/src/osqp_interface.cpp
  )

  # Add include directories
  list(APPEND ${INCLUDE_DIRS}
    src/osqp_interface/include
    "${OSQP_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
  )
endmacro()
