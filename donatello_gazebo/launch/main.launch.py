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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    launch_gazebo_gui_arg = DeclareLaunchArgument(
        name="launch_gazebo_gui",
        description="Set to true to launch the Gazebo GUI",
        choices=["true", "false"],
        default_value="false",
    )
    launch_gazebo_gui_conf = LaunchConfiguration("launch_gazebo_gui")

    gzserver_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r -s -v4 ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("donatello_bookstore_world"),
                        "worlds",
                        "bookstore.sdf",
                    ]
                ),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
        condition=IfCondition(
            PythonExpression(['"', launch_gazebo_gui_conf, '" == "true"']),
        ),
    )

    bridge_params = os.path.join(
        get_package_share_directory("donatello_gazebo"), "config", "bridge.yaml"
    )

    gazebo_ros_bride_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "donatello",
            "-x",
            "0.0",
            "-y",
            "2.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            launch_gazebo_gui_arg,
            gzserver_launch_include,
            gzclient_launch_include,
            gazebo_ros_bride_node,
            robot_spawner_node,
        ]
    )
