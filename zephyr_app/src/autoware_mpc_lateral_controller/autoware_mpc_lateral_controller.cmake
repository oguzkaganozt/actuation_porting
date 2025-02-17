macro(add_autoware_mpc_lateral_controller)
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_mpc_lateral_controller/src/mpc_lateral_controller.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_mpc_lateral_controller/include
  )
endmacro()
