# Copyright (c) 2022-2023, Arm Limited.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(actuation_message_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CycloneDDS REQUIRED COMPONENTS ddsc)

set(${PROJECT_NAME}_NODE_SRC
  src/actuation_message_converter_node.cpp
)

set(${PROJECT_NAME}_NODE_HEADERS
  include/actuation_message_converter/actuation_message_converter_node.hpp
  include/actuation_message_converter/visibility_control.hpp
)

# Generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${${PROJECT_NAME}_NODE_SRC}
  ${${PROJECT_NAME}_NODE_HEADERS}
)

target_link_libraries(${PROJECT_NAME}_node
  CycloneDDS::ddsc
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::actuation_message_converter::MessageConverterNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_actuation_message_converter.cpp)
  set(TEST_ACTUATION_MESSAGE_CONVERTER_EXE test_actuation_message_converter)
  ament_add_gtest(${TEST_ACTUATION_MESSAGE_CONVERTER_EXE} ${TEST_SOURCES})
  target_include_directories(${TEST_ACTUATION_MESSAGE_CONVERTER_EXE} SYSTEM BEFORE PRIVATE
    "${CycloneDDS_CONFIG_DIR}/../../../include/"
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/"
  )
  target_link_libraries(${TEST_ACTUATION_MESSAGE_CONVERTER_EXE}
    ${PROJECT_NAME}_node
  )
  target_compile_options(${TEST_ACTUATION_MESSAGE_CONVERTER_EXE} PRIVATE
    -pthread
  )
endif()

# Ament package generation and installation
ament_auto_package(INSTALL_TO_SHARE
  launch
)
