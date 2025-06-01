#!/usr/bin/python3

# Copyright 2025 Gerardo Puga
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

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os


def generate_launch_description():
    share_dir = get_package_share_directory("donatello_rosbridge")

    config_path_base = os.path.join(share_dir, "config")

    rosbridge_server_config = [os.path.join(config_path_base, "rosbridge_server.yaml")]
    rosapi_node_config = [os.path.join(config_path_base, "rosapi_node.yaml")]

    rosbridge_server_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_server",
        respawn=True,
        parameters=[rosbridge_server_config],
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi_node",
        respawn=True,
        parameters=[rosapi_node_config],
    )

    return LaunchDescription(
        [
            rosbridge_server_node,
            rosapi_node,
        ]
    )
