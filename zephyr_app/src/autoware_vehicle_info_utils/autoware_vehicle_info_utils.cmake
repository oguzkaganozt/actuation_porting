macro(add_autoware_vehicle_info_utils)
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_vehicle_info_utils/src/vehicle_info_utils.cpp
    src/autoware_vehicle_info_utils/src/vehicle_info.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_vehicle_info_utils/include
    src/autoware_universe_utils/include
  )
endmacro()
