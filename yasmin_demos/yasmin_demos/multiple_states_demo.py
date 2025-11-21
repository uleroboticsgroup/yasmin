#!/usr/bin/env python3

# Copyright (C) 2025 Pedro Edom Nunes
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
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_demos import FooState, BarState


def main() -> None:
    yasmin.YASMIN_LOG_INFO("yasmin_multiple_states_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "FOO",
        FooState(),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4",
        },
    )
    sm.add_state(
        "BAR",
        BarState(),
        transitions={
            "outcome3": "FOO",
        },
    )

    # Publish FSM information for visualization
    viewer = YasminViewerPub(sm, "YASMIN_MULTIPLE_STATES_DEMO")

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
