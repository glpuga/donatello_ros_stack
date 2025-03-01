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
        name="rviz", default_value="false", description="Launch RViz"
    )

    rviz_conf = LaunchConfiguration("rviz")

    def build_launch_description_source(pkg):
        return PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", "main.launch.py"])
        )

    launch_description = IncludeLaunchDescription(
        build_launch_description_source("donatello_description")
    )

    launch_rviz = IncludeLaunchDescription(
        build_launch_description_source("donatello_rviz2"),
        condition=IfCondition(rviz_conf),
    )

    launch_teleop = IncludeLaunchDescription(
        build_launch_description_source("donatello_teleop")
    )

    launch_localization = IncludeLaunchDescription(
        build_launch_description_source("donatello_localization")
    )

    launch_navigation = IncludeLaunchDescription(
        build_launch_description_source("donatello_navigation")
    )

    launch_odom_rf2o = IncludeLaunchDescription(
        build_launch_description_source("donatello_rf2o_odom")
    )

    launch_odom_filter = IncludeLaunchDescription(
        build_launch_description_source("donatello_odom_filter")
    )

    launch_misc = IncludeLaunchDescription(
        build_launch_description_source("donatello_misc")
    )

    return LaunchDescription(
        [
            rviz_arg,
            launch_description,
            launch_rviz,
            launch_teleop,
            launch_localization,
            launch_navigation,
            launch_odom_rf2o,
            launch_odom_filter,
            launch_misc,
        ]
    )
