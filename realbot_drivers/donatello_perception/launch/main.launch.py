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

    launch_rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [this_pkg_share, "launch", "driver_rplidar.launch.py"]
            ),
        ),
    )

    # launch_realsense_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [this_pkg_share, "launch", "driver_realsense.launch.py"]
    #         ),
    #     ),
    # )

    launch_robomaster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [this_pkg_share, "launch", "driver_robomaster_ep.launch.py"]
            ),
        ),
    )

    return LaunchDescription(
        [
            launch_rplidar_driver,
            # launch_realsense_driver,
            launch_robomaster_driver,
        ]
    )
