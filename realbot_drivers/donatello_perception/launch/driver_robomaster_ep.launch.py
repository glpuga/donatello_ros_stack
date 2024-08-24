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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch.actions import GroupAction
from launch_ros.actions import SetRemap

from launch_ros.actions import Node


def generate_launch_description():
    robomaster_launchfile_arguments = {
        "name": "robomaster_ep",
        "model": "ep",
        "with_model_description": "false",  # we have our own custom model description
        "log_level": "info",
        "serial_number": "",
        "conn_type": "rndis",
        "lib_log_level": "ERROR",
        "reconnect": "true",
        "chassis_imu_includes_orientation": "false",
        "chassis_twist_to_wheel_speeds": "true",
    }

    launch_robomaster_ros_driver = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robomaster_ros"), "launch", "ep.launch"]
            ),
        ),
        launch_arguments=robomaster_launchfile_arguments.items(),
    )

    # this is is needed because I can't change the tf_prefix parameter in the driver
    # without removing the robomaster_ep namespace from the topics
    static_odom_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "robomaster_ep/odom"],
        output="both",
    )

    return LaunchDescription(
        [
            GroupAction(
                [
                    SetRemap(src="/robomaster_ep/odom", dst="/odom"),
                    SetRemap(src="/robomaster_ep/cmd_vel", dst="/cmd_vel"),
                    launch_robomaster_ros_driver,
                ]
            ),
            static_odom_tf_publisher,
        ]
    )
