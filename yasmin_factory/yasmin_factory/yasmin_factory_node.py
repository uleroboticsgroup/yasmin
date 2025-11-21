#!/usr/bin/env python3

# Copyright (C) 2025 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rclpy
import yasmin
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub
from yasmin_factory import YasminFactory


def main() -> None:
    yasmin.YASMIN_LOG_INFO("yasmin_factory_node")

    # Initialize ROS 2
    rclpy.init()

    # Get the state machine file parameter
    node = YasminNode.get_instance()
    node.declare_parameter("state_machine_file", "")
    sm_file = node.get_parameter("state_machine_file").get_parameter_value().string_value

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    factory = YasminFactory()
    sm = factory.create_sm_from_file(sm_file)

    # Publish FSM information for visualization
    viewer = YasminViewerPub(sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        viewer.cleanup()
        del sm

        # Shutdown ROS 2 if it's running
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
