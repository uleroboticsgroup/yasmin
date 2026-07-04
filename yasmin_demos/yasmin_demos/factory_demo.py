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

import os
import rclpy
import yasmin
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub
from yasmin_factory import YasminFactory
from ament_index_python import get_package_share_directory


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("YASMIN_FACTORY_DEMO")

    # Create a finite state machine (FSM)
    factory = YasminFactory()
    sm = factory.create_sm_from_file(
        os.path.join(
            get_package_share_directory("yasmin_demos"), "state_machines", "demo_1.xml"
        )
    )
    sm.set_sigint_handler(True)

    # Publish FSM information for visualization
    pub = YasminViewerPub(sm, "plugin_demo")

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
    finally:
        pub.shutdown()

    # Shutdown ROS 2 if it's running
    YasminNode.destroy_instance()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
