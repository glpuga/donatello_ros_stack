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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg_share = FindPackageShare("donatello_perception")

    rviz_config_filepath = PathJoinSubstitution(
        [this_pkg_share, "rviz", "display.rviz"]
    )

    urdf_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "urdf_package": "donatello_description",
            "urdf_package_path": "urdf/donatello.urdf.xacro",
            "rviz_config": rviz_config_filepath,
            "jsp_gui": "false",
        }.items(),
    )

    launch_rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [this_pkg_share, "launch", "driver_rplidar.launch.py"]
            ),
        ),
    )

    launch_realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [this_pkg_share, "launch", "driver_realsense.launch.py"]
            ),
        ),
    )

    return LaunchDescription(
        [
            urdf_launch_include,
            launch_rplidar_driver,
            launch_realsense_driver,
        ]
    )
