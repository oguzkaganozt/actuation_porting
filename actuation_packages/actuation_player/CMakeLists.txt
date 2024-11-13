# Copyright (c) 2023, Arm Limited.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.12)

project(actuation_player)
include(GNUInstallDirs)

# Require that dependencies from package.xml be available
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CycloneDDS REQUIRED COMPONENTS ddsc)

set(${PROJECT_NAME}_HEADERS
  include/actuation_player/actuation_player.hpp
)

# Library
ament_auto_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_HEADERS}
  src/actuation_player.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  SOVERSION 1
)

target_link_libraries(${PROJECT_NAME}
  CycloneDDS::ddsc
)

# Executable
ament_auto_add_executable(${PROJECT_NAME}_exe
  ${${PROJECT_NAME}_HEADERS}
  src/actuation_player_main.cpp
)

set_target_properties(${PROJECT_NAME}_exe PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
    ${${PROJECT_NAME}_HEADERS}
    test/test_actuation_player.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/test_data/
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

# Install
install(TARGETS ${PROJECT_NAME} LIBRARY)
install(TARGETS ${PROJECT_NAME}_exe RUNTIME)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

# Ament package generation and installation
ament_package()
