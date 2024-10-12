#!/usr/bin/python3

# Copyright 2023 Ekumen, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.utilities.type_utils import get_typed_value
from launch import LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.utilities import perform_substitutions
from launch.substitutions import LaunchConfiguration
from typing import List
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def with_launch_arguments(arguments_to_declare: List[DeclareLaunchArgument]):
    """Decorate generate_launch_description function to resolve launch arguments early."""

    def decorator(get_launch_description_f):
        def wrapper():
            launch_configurations_for_arguments = [
                LaunchConfiguration(arg.name) for arg in arguments_to_declare
            ]

            def action(context):
                resolved_args = {
                    perform_substitutions(
                        context, config.variable_name
                    ): config.perform(context)
                    for config in launch_configurations_for_arguments
                }
                return [
                    IncludeLaunchDescription(
                        LaunchDescriptionSource(
                            get_launch_description_f(**resolved_args)
                        )
                    )
                ]

            return LaunchDescription(
                [
                    *arguments_to_declare,
                    OpaqueFunction(function=action),
                ]
            )

        return wrapper

    return decorator


def get_launch_arguments():
    return [
        DeclareLaunchArgument(
            name="start_paused",
            default_value="False",
            description="Start the rosbag player in a paused state.",
        ),
        DeclareLaunchArgument(
            name="rate",
            default_value="1.",
            description="Rate used to playback the bag.",
        ),
        DeclareLaunchArgument(
            name="loop",
            choices=["True", "False"],
            default_value="True",
            description="Rate used to playback the bag.",
        ),
        DeclareLaunchArgument(
            name="file_path",
            description="Full path to the bag to play back.",
        ),
    ]


@with_launch_arguments(get_launch_arguments())
def generate_launch_description(
    start_paused,
    rate,
    loop,
    file_path,
):
    start_paused = get_typed_value(start_paused, bool)
    loop = get_typed_value(loop, bool)

    bag_play_cmd = [
        "xterm",
        "-e",
        "ros2",
        "bag",
        "play",
        file_path,
        "--rate",
        rate,
        "--clock",
    ]

    if start_paused:
        bag_play_cmd.append("--start-paused")

    if loop:
        bag_play_cmd.append("--loop")

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    ExecuteProcess(
                        cmd=bag_play_cmd,
                        output="own_log",
                        on_exit=[Shutdown()],
                    ),
                ]
            ),
            GroupAction(
                actions=[
                    SetParameter(
                        name="use_sim_time",
                        value=True,
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("donatello_rviz2"),
                                    "launch",
                                    "main.launch.py",
                                ]
                            )
                        ),
                    ),
                ],
            ),
        ]
    )
