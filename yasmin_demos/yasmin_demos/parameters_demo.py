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

import time

import rclpy
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

import yasmin
from yasmin import Blackboard, State, StateMachine
from yasmin_ros import GetParametersState, set_ros_loggers
from yasmin_viewer import YasminViewerPub


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0
        self.set_description(
            "Increments a counter until the value from 'max_counter' is reached and writes the formatted counter string to the blackboard."
        )
        self.add_input_key(
            "max_counter",
            "Maximum number of iterations before the state finishes.",
        )
        self.add_input_key(
            "counter_str",
            "Prefix used when formatting the counter string.",
            "Counter",
        )
        self.add_output_key(
            "foo_str",
            "Formatted counter string written to the blackboard.",
        )

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(3)  # Simulate work by sleeping

        if self.counter < blackboard["max_counter"]:
            self.counter += 1
            blackboard["foo_str"] = f"{blackboard['counter_str']}: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# Define the BarState class, inheriting from the State class
class BarState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: Indicates the state should transition back to the Foo state.
        """
        super().__init__(outcomes=["outcome3"])
        self.set_description(
            "Reads and prints the formatted counter string stored in 'foo_str' from the blackboard."
        )
        self.add_input_key(
            "foo_str",
            "Formatted counter string produced by FooState.",
        )

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome3".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        time.sleep(3)  # Simulate work by sleeping

        yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        return "outcome3"


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_parameters_demo")

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"], handle_sigint=True)
    sm.set_description(
        "Loads configuration values into the blackboard and then runs a loop that formats and prints a counter string until the configured maximum is reached."
    )
    sm.add_input_key(
        "max_counter",
        "Maximum number of iterations before the state machine finishes.",
        3,
    )
    sm.add_input_key(
        "counter_str",
        "Prefix used when formatting the counter string.",
        "Counter",
    )
    sm.add_output_key(
        "foo_str",
        "Formatted counter string produced during execution.",
    )

    getting_parameters_state = GetParametersState(
        parameters={
            "max_counter": 3,
            "counter_str": "Counter",
        },
    )
    getting_parameters_state.set_description(
        "Loads the configured parameters and stores them in the blackboard."
    )
    getting_parameters_state.add_output_key(
        "max_counter",
        "Maximum number of iterations before the state machine finishes.",
    )
    getting_parameters_state.add_output_key(
        "counter_str",
        "Prefix used when formatting the counter string.",
    )

    # Add states to the FSM
    sm.add_state(
        "GETTING_PARAMETERS",
        getting_parameters_state,
        transitions={
            SUCCEED: "FOO",
            ABORT: "outcome4",
        },
    )

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
    YasminViewerPub(sm, "YASMIN_PARAMETERS_DEMO")

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
