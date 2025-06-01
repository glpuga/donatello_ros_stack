#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import random


class TwistDitherNode(Node):
    def __init__(self):
        """Create a TwistDitherNode."""
        super().__init__("donatello_twist_dithering_node")

        self.declare_parameter("robot_hysteresis", [0.1, 0.1, 0.5])
        self.declare_parameter("virtual_hysteresis", [0.01, 0.01, 0.05])
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter(
            "mode", "round_outwards"
        )  # Options: passthrough, round_outwards, natural_deadband, dither_deadband
        self.robot_hysteresis_ = (
            self.get_parameter("robot_hysteresis")
            .get_parameter_value()
            .double_array_value
        )
        self.virtual_hysteresis_ = (
            self.get_parameter("virtual_hysteresis")
            .get_parameter_value()
            .double_array_value
        )
        self.rate_ = self.get_parameter("frequency").get_parameter_value().double_value

        self.mode_ = self.get_parameter("mode").get_parameter_value().string_value

        def round_outwards(value, robot_hysteresis, virtual_hysteresis):
            module = abs(value)
            # If the value is greater than the robot hysteresis, do nothing.
            if module > robot_hysteresis:
                return value
            # If the value is smaller than the virtual hysteresis, we will set it to 0.
            if module < virtual_hysteresis:
                return 0.0
            # if it's in the middle ground, round it to the robot hysteresis.
            sign = 1.0 if value > 0.0 else -1.0
            return robot_hysteresis * sign

        def natural_deadband(value, robot_hysteresis, virtual_hysteresis):
            module = abs(value)
            if module > robot_hysteresis:
                return value
            return 0.0

        def dither_deadband(value, robot_hysteresis, virtual_hysteresis):
            module = abs(value)
            if module > robot_hysteresis:
                return value
            if module < virtual_hysteresis:
                return 0.0
            sign = 1.0 if value > 0.0 else -1.0
            # For anything in the middle, we will dither the value by time averaging.
            p = module / robot_hysteresis
            return robot_hysteresis * sign if p > random.random() else 0.0

        # default to doing nothing
        if self.mode_ == "passthrough":
            self.dither_value = (
                lambda value, robot_hysteresis, virtual_hysteresis: value
            )
        elif self.mode_ == "round_outwards":
            self.dither_value = round_outwards
        elif self.mode_ == "natural_deadband":
            self.dither_value = natural_deadband
        elif self.mode_ == "dither_deadband":
            self.dither_value = dither_deadband
        else:
            self.get_logger().error(
                "No distortion mode selected. Failing fast by refusing to move the robot"
            )
            self.dither_value = lambda value, robot_hysteresis, virtual_hysteresis: 0.0

        self.get_logger().info(
            "Robot hysteresis: {}".format(
                self.get_parameter("timeout").get_parameter_value().double_value
            )
        )

        self.timeout_ = rclpy.duration.Duration(
            seconds=self.get_parameter("timeout").get_parameter_value().double_value
        )

        self.timer_period_ = 1.0 / self.rate_

        self.latest_message_ = None
        self.latest_message_time_ = None

        self.goal_publisher_ = self.create_publisher(Twist, "cmd_vel_out", 1)
        self.current_pose_subscriber_ = self.create_subscription(
            Twist, "cmd_vel_in", self.process_twist_command, 1
        )

        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

    def timer_callback(self):
        """Trigger the publication of the output message."""
        self.publish_output_msg()

    def process_twist_command(self, msg):
        """Process the incoming Twist message."""
        self.latest_message_ = msg
        self.latest_message_time_ = self.get_clock().now()
        self.publish_output_msg()

    def publish_output_msg(self):
        """Dither the incoming Twist message and republish it."""
        if self.latest_message_ is None:
            return
        if self.latest_message_time_ - self.get_clock().now() > self.timeout_:
            self.latest_message_ = None
            self.get_logger().error("cmd_vel command timed out.")
            return
        output_message = Twist()
        output_message.linear.x = self.dither_value(
            self.latest_message_.linear.x,
            self.robot_hysteresis_[0],
            self.virtual_hysteresis_[0],
        )
        output_message.linear.y = self.dither_value(
            self.latest_message_.linear.y,
            self.robot_hysteresis_[1],
            self.virtual_hysteresis_[1],
        )
        output_message.angular.z = self.dither_value(
            self.latest_message_.angular.z,
            self.robot_hysteresis_[2],
            self.virtual_hysteresis_[2],
        )
        self.goal_publisher_.publish(output_message)


def main(args=None):
    rclpy.init(args=args)
    node_handle = TwistDitherNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
