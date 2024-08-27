<!--
    # Based on: https://github.com/autowarefoundation/autoware_common/blob/ba4d3cc3f729ebedc16aba7d745bcc54fd935e61/autoware_cmake/README.md
    # In open-source project: autoware_common
    # Original file: Copyright (c) 2022, The Autoware Contributors
    # Modifications: Copyright (c) 2023, Arm Limited.
    # SPDX-License-Identifier: Apache-2.0
-->

# autoware_cmake

This package provides CMake scripts for Autoware.

## Usage

### autoware_package.cmake

Call `autoware_package()` before defining build targets, which will set common options for Autoware.

```cmake
cmake_minimum_required(VERSION 3.5)
project(package_name)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(...)
```
