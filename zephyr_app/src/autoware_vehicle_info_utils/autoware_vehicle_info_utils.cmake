macro(add_autoware_vehicle_info_utils SRCS INCLUDE_DIRS)
  # Add source files
  list(APPEND ${SRCS}
    src/autoware_vehicle_info_utils/src/vehicle_info_utils.cpp
    src/autoware_vehicle_info_utils/src/vehicle_info.cpp
  )

  # Add include directories
  list(APPEND ${INCLUDE_DIRS}
    src/autoware_vehicle_info_utils/include
    src/autoware_universe_utils/include
  )
endmacro()
