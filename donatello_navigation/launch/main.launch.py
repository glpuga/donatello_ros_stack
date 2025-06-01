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
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_package_share = get_package_share_directory('donatello_navigation')
    config_package_share = get_package_share_directory(
        'donatello_navigation_configurations'
    )

    planner_configs_install_folder = os.path.join(
        config_package_share,
        'planner_config',
    )
    controller_configs_install_folder = os.path.join(
        config_package_share,
        'controller_config',
    )

    available_planners = os.listdir(planner_configs_install_folder)
    available_controllers = os.listdir(controller_configs_install_folder)

    planner_conf, argument_declaration_planner = (
        LaunchConfiguration(
            'nav_planner',
        ),
        DeclareLaunchArgument(
            name='nav_planner',
            description='Planner to use for navigation',
            choices=available_planners,
            default_value='hybrid-a',
        ),
    )

    controller_conf, argument_declaration_controller = (
        LaunchConfiguration(
            'nav_controller',
        ),
        DeclareLaunchArgument(
            name='nav_controller',
            description='Controller to use for navigation',
            choices=available_controllers,
            default_value='regulated_pure_pursuit',
        ),
    )

    this_package_param_files = [
        'behavior_server_params.yaml',
        'bt_navigator_params.yaml',
        'hystheresis_control_params.yaml',
        'smoother_server_params.yaml',
        'velocity_smoother_params.yaml',
        'waypoint_follower_params.yaml',
    ]

    overall_params = [
        os.path.join(this_package_share, 'config', name)
        for name in this_package_param_files
    ]

    planner_config_files = [
        'global_costmap_params.yaml',
        'planner_server_params.yaml',
    ]

    controller_config_files = [
        'controller_server_params.yaml',
        'local_costmap_params.yaml',
    ]

    overall_params += [
        PathJoinSubstitution(
            [
                planner_configs_install_folder,
                planner_conf,
                file,
            ]
        )
        for file in planner_config_files
    ] + [
        PathJoinSubstitution(
            [
                controller_configs_install_folder,
                controller_conf,
                file,
            ]
        )
        for file in controller_config_files
    ]

    common_node_arguments = {
        'output': 'screen',
        'respawn': True,
        'respawn_delay': 2.0,
        'parameters': overall_params,
        'arguments': ['--ros-args', '--log-level', 'info'],
    }

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        remappings=[
            ('/cmd_vel', '/cmd_vel_controller'),
        ],
        **common_node_arguments,
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        **common_node_arguments,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        **common_node_arguments,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        remappings=[
            ('/cmd_vel', '/cmd_vel_controller'),
        ],
        **common_node_arguments,
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        **common_node_arguments,
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        **common_node_arguments,
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        remappings=[
            ('/cmd_vel', '/cmd_vel_mux'),
            ('/cmd_vel_smoothed', '/cmd_vel_raw'),
        ],
        **common_node_arguments,
    )

    hystheresis_control_node = Node(
        package='donatello_twist_dithering',
        executable='twist_dithering',
        name='hystheresis_control_node',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_raw'),
            ('/cmd_vel_out', '/cmd_vel'),
        ],
        **common_node_arguments,
    )

    lifecycle_node_names = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        respawn=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_node_names},
        ],
    )

    return LaunchDescription(
        [
            argument_declaration_controller,
            argument_declaration_planner,
            behavior_server_node,
            bt_navigator_node,
            controller_server_node,
            hystheresis_control_node,
            lifecycle_manager_node,
            planner_server_node,
            smoother_server_node,
            velocity_smoother_node,
            waypoint_follower_node,
        ]
    )
