
# Add source files
list(APPEND APP_SOURCES
  src/autoware/autoware_mpc_lateral_controller/src/mpc_lateral_controller.cpp
  src/autoware/autoware_mpc_lateral_controller/src/mpc.cpp
  src/autoware/autoware_mpc_lateral_controller/src/mpc_trajectory.cpp
  src/autoware/autoware_mpc_lateral_controller/src/mpc_utils.cpp
  src/autoware/autoware_mpc_lateral_controller/src/steering_predictor.cpp
  src/autoware/autoware_mpc_lateral_controller/src/lowpass_filter.cpp
  src/autoware/autoware_mpc_lateral_controller/src/qp_solver/qp_solver_unconstraint_fast.cpp
  src/autoware/autoware_mpc_lateral_controller/src/steering_offset/steering_offset.cpp
  src/autoware/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/autoware/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/autoware/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/autoware/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_interface.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/autoware/autoware_mpc_lateral_controller/include
)
