macro(add_osqp_interface SRCS INCLUDE_DIRS)
  find_package(Eigen3 REQUIRED)
  find_package(osqp REQUIRED)

  get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
  get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

  set(OSQP_INTERFACE_LIB_SRC
    src/osqp_interface/src/csc_matrix_conv.cpp
    src/osqp_interface/src/osqp_interface.cpp
  )

  set(OSQP_INTERFACE_INCLUDE_DIRS
    src/osqp_interface/include
    src/osqp_interface/include
    src/osqp_interface/include
    "${OSQP_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
  )

  # Add source files
  list(APPEND ${SRCS} ${OSQP_INTERFACE_LIB_SRC})

  # Add include directories
  list(APPEND ${INCLUDE_DIRS} ${OSQP_INTERFACE_LIB_HEADERS})

endmacro()
