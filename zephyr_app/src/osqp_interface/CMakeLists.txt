cmake_minimum_required(VERSION 3.5)
project(osqp_interface)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

find_package(ament_cmake_auto REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
# ament_auto_find_build_dependencies()

get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

set(OSQP_INTERFACE_LIB_SRC
  src/csc_matrix_conv.cpp
  src/osqp_interface.cpp
)

set(OSQP_INTERFACE_LIB_HEADERS
  include/osqp_interface/csc_matrix_conv.hpp
  include/osqp_interface/osqp_interface.hpp
  include/osqp_interface/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME}
  ${OSQP_INTERFACE_LIB_SRC}
  ${OSQP_INTERFACE_LIB_HEADERS}
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    "${OSQP_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
)

ament_auto_package()
