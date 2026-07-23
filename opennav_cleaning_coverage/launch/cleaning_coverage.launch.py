# Copyright 2026 Open Navigation LLC
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

"""Launch the cleaning coverage server and its lifecycle manager."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("opennav_cleaning_coverage")
    default_params = os.path.join(
        package_share, "config", "cleaning_coverage.yaml")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Cleaning coverage server parameter file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Configure and activate the lifecycle node",
            ),
            Node(
                package="opennav_cleaning_coverage",
                executable="cleaning_coverage_server",
                name="cleaning_coverage_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_cleaning_coverage",
                output="screen",
                parameters=[
                    {
                        "autostart": autostart,
                        # rclpy.lifecycle.LifecycleNode does not create the
                        # Nav2 bond expected by the lifecycle manager.
                        "bond_timeout": 0.0,
                        "node_names": ["cleaning_coverage_server"],
                    }
                ],
            ),
        ]
    )
