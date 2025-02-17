macro(add_autoware_motion_utils)
  # Add source files
  list(APPEND APP_SOURCES
    src/autoware_motion_utils/src/motion_utils.cpp
  )

  # Add include directories
  list(APPEND APP_INCLUDE_DIRS
    src/autoware_motion_utils/include
  )
endmacro()
