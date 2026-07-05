#!/usr/bin/env python3

# Copyright (C) 2025 Miguel Ángel González Santamarta
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
from yasmin_ros.yasmin_node import YasminNode

import yasmin
from yasmin_factory import YasminFactory
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Get the state machine file parameter
    node = YasminNode.get_instance()
    node.declare_parameter("state_machine_file", "")
    sm_file = node.get_parameter("state_machine_file").get_parameter_value().string_value

    # Get if enable viewer parameter
    node.declare_parameter("enable_viewer_pub", True)
    enable_viewer_pub = (
        node.get_parameter("enable_viewer_pub").get_parameter_value().bool_value
    )

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_factory_node")

    # Create a finite state machine (FSM)
    factory = YasminFactory()
    sm = factory.create_sm_from_file(sm_file)
    sm.set_sigint_handler(True)

    # Publish FSM information for visualization
    pub = None
    if enable_viewer_pub:
        pub = YasminViewerPub(sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if pub is not None:
        pub.shutdown()

    YasminNode.destroy_instance()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
