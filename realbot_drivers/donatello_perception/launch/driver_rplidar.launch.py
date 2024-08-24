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
    rplidar_a1_m12_driver_arguments = {
        "serial_port": "/dev/rplidar",
        "frame_id": "rear_rplidar_optical_link",
        "inverted": "false",
        "angle_compensate": "true",
        "scan_mode": "Sensitivity",
        "channel_type": "serial",
        "serial_baudrate": "256000",
    }

    launch_rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rplidar_ros"), "launch", "rplidar_a2m12_launch.py"]
            ),
        ),
        launch_arguments=rplidar_a1_m12_driver_arguments.items(),
    )

    return LaunchDescription([launch_rplidar_driver])
