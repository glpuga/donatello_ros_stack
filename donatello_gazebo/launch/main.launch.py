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

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world_name = "minimal.sdf"

    gzserver_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_args": [
                "-r -s -v4 ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("donatello_gazebo"),
                        "worlds",
                        default_world_name,
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

    # robomaster_description is needed because the meshes from the URDF are
    # loaded from there, but it's not a direct dependency. This is a workaround
    # to make sure the meshes are found because of the way Gazebo finds resources.

    # TODO: remove this once the mecanum drive is working
    paths_to_resources = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(
                Path(
                    os.path.join(get_package_share_directory("robomaster_description"))
                ).parent.resolve()
            ),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(
                Path(
                    os.path.join(get_package_share_directory("donatello_description"))
                ).parent.resolve()
            ),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(
                Path(
                    os.path.join(get_package_share_directory("donatello_gazebo"))
                ).parent.resolve()
            ),
        ),
    ]

    return LaunchDescription(
        [
            *paths_to_resources,
            ExecuteProcess(
                cmd=["env"],
                output="screen",
            ),
            gzserver_launch_include,
            gzclient_launch_include,
            gazebo_ros_bride_node,
            robot_spawner_node,
        ]
    )
