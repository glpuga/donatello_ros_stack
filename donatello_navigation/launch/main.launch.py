#!/usr/bin/env python3

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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO - This should be a parameter
    controller_type = "mppi_omni"

    this_package_share = get_package_share_directory("donatello_navigation")

    parameter_files = [
        "behavior_server_params.yaml",
        "bt_navigator_params.yaml",
        "controller_params_{}.yaml".format(controller_type),
        "global_costmap_params.yaml",
        "local_costmap_params.yaml",
        "planner_server_params.yaml",
        "smoother_server_params.yaml",
        "velocity_smoother_params.yaml",
        "waypoint_follower_params.yaml",
    ]

    overall_params = [
        os.path.join(this_package_share, "config", name) for name in parameter_files
    ]

    common_node_arguments = {
        "output": "screen",
        "respawn": True,
        "respawn_delay": 2.0,
        "parameters": overall_params,
        "arguments": ["--ros-args", "--log-level", "info"],
        "remappings": [
            ('/cmd_vel', '/cmd_vel_controller'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ],
    }

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        **common_node_arguments,
    )

    smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        **common_node_arguments,
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        **common_node_arguments,
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        **common_node_arguments,
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        **common_node_arguments,
    )

    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        **common_node_arguments,
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        **common_node_arguments,
    )

    lifecycle_node_names = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        respawn=True,
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"autostart": True},
            {"node_names": lifecycle_node_names},
        ],
    )

    return LaunchDescription(
        [
            controller_server_node,
            smoother_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            lifecycle_manager_node,
            velocity_smoother_node,
        ]
    )