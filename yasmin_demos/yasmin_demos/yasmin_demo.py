#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta
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

import yasmin
from yasmin import StateMachine
from yasmin_demos.bar_state import BarState
from yasmin_demos.foo_state import FooState
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"], handle_sigint=True)
    sm.set_description(
        "Runs a simple loop between FooState and BarState until FooState reaches its terminal outcome."
    )
    sm.add_output_key(
        "foo_str",
        "Formatted counter string produced by FooState and read by BarState.",
    )

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
    pub = YasminViewerPub(sm, "YASMIN_DEMO")

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
