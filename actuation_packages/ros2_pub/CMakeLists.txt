
cmake_minimum_required(VERSION 3.20)

project(ros2_pub)

# Require that dependencies from package.xml be available
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/ros2_pub.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Ament package generation and installation
ament_auto_package()
