#!/usr/bin/python3

# Copyright 2024 Gerardo Puga
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

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

import os


def generate_launch_description():
    beluga_amcl_tag = "beluga_amcl"
    nav2_amcl_tag = "nav2_amcl"
    no_localization_tag = "none"

    this_package_share = get_package_share_directory("donatello_localization")

    maps_install_folder = os.path.join(this_package_share, "maps")
    available_maps = os.listdir(maps_install_folder)

    localization_package_arg = DeclareLaunchArgument(
        name="localization_package",
        description="Localization packages to use, if any",
        choices=[
            beluga_amcl_tag,
            nav2_amcl_tag,
            no_localization_tag,
        ],
        default_value=beluga_amcl_tag,
    )
    localization_package_conf = LaunchConfiguration("localization_package")

    map_name_arg = DeclareLaunchArgument(
        name="map_name",
        description="Name of the localization map to use. Must be one in the maps folder.",
        choices=available_maps,
        default_value=available_maps[0],
    )
    map_name_conf = LaunchConfiguration("map_name")

    localization_params_file = os.path.join(this_package_share, "config", "params.yaml")

    localization_map = PathJoinSubstitution(
        [maps_install_folder, map_name_conf, "map.yaml"]
    )

    common_node_arguments = {
        "name": "amcl",
        "output": "screen",
        "arguments": ["--ros-args", "--log-level", "info"],
        "respawn": True,
        "parameters": [localization_params_file],
    }

    conditioned_amcl_node = Node(
        package="beluga_amcl",
        executable="amcl_node",
        condition=IfCondition(
            PythonExpression(
                ['"', localization_package_conf, f'" == "{beluga_amcl_tag}"']
            ),
        ),
        **common_node_arguments,
    )

    conditioned_nav2_amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        condition=IfCondition(
            PythonExpression(
                ['"', localization_package_conf, f'" == "{nav2_amcl_tag}"']
            ),
        ),
        **common_node_arguments,
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        respawn=True,
        parameters=[{"yaml_filename": localization_map}],
        condition=IfCondition(
            PythonExpression(
                ['"', localization_package_conf, f'" != "{no_localization_tag}"']
            ),
        ),
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        sigterm_timeout="20",
        sigkill_timeout="20",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    return LaunchDescription(
        [
            localization_package_arg,
            map_name_arg,
            conditioned_amcl_node,
            conditioned_nav2_amcl_node,
            map_server_node,
            lifecycle_manager_node,
        ]
    )
