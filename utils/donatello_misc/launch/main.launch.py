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

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

import os


def generate_launch_description():
    share_dir = get_package_share_directory("donatello_misc")

    cmd_vel_mux_params_file = os.path.join(
        share_dir, "config", "cmd_vel_mux_params.yaml"
    )

    remappings = {"/cmd_vel_out": "/cmd_vel_mux"}

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="cmd_vel_mux",
        output="both",
        parameters=[cmd_vel_mux_params_file],
        remappings=remappings.items(),
    )

    return LaunchDescription([twist_mux_node])
