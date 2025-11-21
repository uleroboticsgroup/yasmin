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

import os
import rclpy
import yasmin
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_factory import YasminFactory
from ament_index_python import get_package_share_directory


def main() -> None:
    yasmin.YASMIN_LOG_INFO("YASMIN_FACTORY_DEMO")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    factory = YasminFactory()
    sm = factory.create_sm_from_file(
        os.path.join(
            get_package_share_directory("yasmin_demos"), "state_machines", "demo_1.xml"
        )
    )

    # Publish FSM information for visualization
    viewer = YasminViewerPub(sm, "plugin_demo")

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
