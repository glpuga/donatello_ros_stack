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

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    rf2o_laser_odom_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        parameters=[
            {
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'base_frame_id': 'donatello/base_link',
                'odom_frame_id': 'donatello/odom',
                'publish_tf': False,
                'init_pose_from_topic': '',  # Keep blank to disable
                'freq': 10.0,
            }
        ],
    )

    return LaunchDescription([rf2o_laser_odom_node])
