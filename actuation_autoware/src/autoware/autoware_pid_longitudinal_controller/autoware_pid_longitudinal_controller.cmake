
# Add source files
list(APPEND APP_SOURCES
  src/autoware/autoware_pid_longitudinal_controller/src/pid.cpp
  src/autoware/autoware_pid_longitudinal_controller/src/smooth_stop.cpp
  src/autoware/autoware_pid_longitudinal_controller/src/longitudinal_controller_utils.cpp
  src/autoware/autoware_pid_longitudinal_controller/src/pid_longitudinal_controller.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/autoware/autoware_pid_longitudinal_controller/include
)
