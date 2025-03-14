# Copyright (c) 2021-2023, Arm Limited., the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# SPDX-License-Identifier: Apache-2.0


"""Note: Does not work in ROS2 dashing."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a single component."""
    container = ComposableNodeContainer(
            name='actuation_message_converter_container',
            namespace='vehicle',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='actuation_message_converter',
                    plugin='autoware::actuation_message_converter::MessageConverterNode',
                    name='actuation_message_converter_node',
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
