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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gamepad_type_conf = LaunchConfiguration("gamepad_type")

    gamepad_dev_conf = LaunchConfiguration("gamepad_device")

    gamepad_type_arg = DeclareLaunchArgument(
        "gamepad_type",
        default_value="xbox",
        choices=["xbox"],
        description="Gamepad type",
    )

    gamepad_dev_arg = DeclareLaunchArgument(
        "gamepad_device",
        default_value="/dev/input/js0",
        description="Gamepad device file path",
    )

    this_package_share = get_package_share_directory("donatello_teleop")

    teleop_twist_joy_config_file = [
        this_package_share,
        "/gamepads/",
        gamepad_type_conf,
        ".config.yaml",
    ]

    joy_linux_config_file = [this_package_share, "/config/joy_linux.config.yaml"]

    gamepad_driver_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[
            {
                "dev": gamepad_dev_conf,
            },
            joy_linux_config_file,
        ],
        remappings=[
            ("/joy", "/teleop/joystick"),
        ],
    )

    teleop_twist_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            teleop_twist_joy_config_file,
        ],
        remappings={
            ("/joy", "/teleop/joystick"),
            ("/cmd_vel", "/cmd_vel_teleop"),
        },
    )

    return LaunchDescription(
        [
            gamepad_type_arg,
            gamepad_dev_arg,
            gamepad_driver_node,
            teleop_twist_node,
        ]
    )
