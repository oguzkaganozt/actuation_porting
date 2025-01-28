cmake_minimum_required(VERSION 3.14)
project(autoware_mpc_lateral_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(steering_offset_lib SHARED
  src/steering_offset/steering_offset.cpp
)

set(MPC_LAT_CON_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${MPC_LAT_CON_LIB} SHARED
  src/mpc_lateral_controller.cpp
  src/lowpass_filter.cpp
  src/steering_predictor.cpp
  src/mpc.cpp
  src/mpc_trajectory.cpp
  src/mpc_utils.cpp
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstraint_fast.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_interface.cpp
)
target_link_libraries(${MPC_LAT_CON_LIB} steering_offset_lib)

ament_auto_package(INSTALL_TO_SHARE
  param
)
