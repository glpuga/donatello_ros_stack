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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        name="use_rviz", default_value="true", description="Launch RViz"
    )

    rviz_conf = LaunchConfiguration("use_rviz")

    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("donatello_description"),
                        "launch",
                        "main.launch.py",
                    ]
                ),
            ]
        ),
    )

    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("donatello_rviz2"), "launch", "main.launch.py"]
                )
            ]
        ),
        condition=IfCondition(rviz_conf),
    )

    return LaunchDescription(
        [
            rviz_arg,
            launch_description,
            launch_rviz,
        ]
    )
