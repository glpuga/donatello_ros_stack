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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    goal_timeout_arg = DeclareLaunchArgument(
        name="goal_timeout",
        default_value="30.0",
        description="Time in seconds to wait for a goal to be reached before sending the next one",
    )
    timer_interval_arg = DeclareLaunchArgument(
        name="timer_period",
        default_value="1.0",
        description="Time in seconds between goal checks",
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        name="goal_tolerance",
        default_value="1.0",
        description="Distance in meters to consider a goal as reached",
    )
    goals_file_arg = DeclareLaunchArgument(
        name="goals_file",
        default_value="donatello_bookstore_poses.yaml",
        description="Path to a yaml file with a list of goals, local to this",
    )

    goal_timeout_conf = LaunchConfiguration("goal_timeout")
    timer_period_conf = LaunchConfiguration("timer_period")
    goal_tolerance_conf = LaunchConfiguration("goal_tolerance")
    goals_file_conf = LaunchConfiguration("goals_file")

    this_package_share = FindPackageShare("donatello_go_for_a_walk")
    goals_file_full_path = PathJoinSubstitution(
        [this_package_share, "config", goals_file_conf]
    )

    random_walk_node = Node(
        package="random_walker",
        executable="random_walker",
        name="random_walker",
        parameters=[
            {"goal_timeout": goal_timeout_conf},
            {"timer_period": timer_period_conf},
            {"goal_tolerance": goal_tolerance_conf},
            {"goals_file_path": goals_file_full_path},
        ],
    )

    return LaunchDescription(
        [
            goal_timeout_arg,
            timer_interval_arg,
            goal_tolerance_arg,
            goals_file_arg,
            random_walk_node,
        ]
    )
