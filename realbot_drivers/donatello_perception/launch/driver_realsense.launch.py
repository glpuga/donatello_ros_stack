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

import os


def generate_launch_description():
    this_package_share = get_package_share_directory("donatello_perception")

    params_file_path = os.path.join(
        this_package_share, "config", "realsense_d435i_config.yaml"
    )

    realsense_driver_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera_driver",
        namespace="camera",
        parameters=[params_file_path],
        output="both",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
    )

    return LaunchDescription([realsense_driver_node])