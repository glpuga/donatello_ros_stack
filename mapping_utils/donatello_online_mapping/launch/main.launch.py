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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        choices=[
            "true",
            "false",
        ],
        description="Use simulation/rosbag clock instead of the system clock",
    )

    use_sim_time_conf = LaunchConfiguration("use_sim_time")

    path_to_config = PathJoinSubstitution(
        [
            FindPackageShare("donatello_online_mapping"),
            "config",
            "online_async_mapping.yaml",
        ]
    )

    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time_conf,
            "slam_params_file": path_to_config,
        }.items(),
    )

    return LaunchDescription([use_sim_time_arg, launch_slam_toolbox])
