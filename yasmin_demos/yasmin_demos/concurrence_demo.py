#!/usr/bin/env python3

# Copyright (C) 2025 Georgia Tech Research Institute
# Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
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

import yasmin
from yasmin import Blackboard, Concurrence, State, StateMachine
from yasmin_ros import set_ros_loggers
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
            outcome1: Indicates the state should continue.
            outcome2: Indicates the state should continue.
            outcome3: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2", "outcome3"])
        self.counter = 0
        self.set_description(
            "Produces a counter string and stores it in the blackboard while cycling through outcomes based on the internal counter."
        )
        self.add_output_key(
            "foo_str",
            "String containing the current counter value produced by FooState.",
        )

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(2)  # Simulate work by sleeping

        blackboard["foo_str"] = f"Counter: {self.counter}"

        if self.counter < 3:
            outcome = "outcome1"
        elif self.counter < 5:
            outcome = "outcome2"
        else:
            outcome = "outcome3"

        yasmin.YASMIN_LOG_INFO("Finishing state FOO")
        self.counter += 1
        return outcome


# Define the BarState class, inheriting from the State class
class BarState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: This state will always return this outcome
        """
        super().__init__(outcomes=["outcome3"])
        self.set_description(
            "Reads and prints the value stored in 'foo_str' from the blackboard."
        )
        self.add_input_key(
            "foo_str",
            "String produced by FooState containing the counter value.",
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
        time.sleep(4)  # Simulate work by sleeping

        if "foo_str" in blackboard:
            yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        else:
            yasmin.YASMIN_LOG_INFO("Blackboard does not yet contain 'foo_str'")

        yasmin.YASMIN_LOG_INFO("Finishing state BAR")

        return "outcome3"


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_concurrence_demo")

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"], handle_sigint=True)
    sm.set_description(
        "Runs FooState and BarState concurrently until the concurrence state no longer matches the configured outcome map."
    )
    sm.add_output_key(
        "foo_str",
        "String containing the current counter value produced during the concurrent execution.",
    )

    # Create states to run concurrently
    foo_state: State = FooState()
    bar_state: State = BarState()

    # Add concurrence state
    concurrence_state = Concurrence(
        states={
            "FOO": foo_state,
            "BAR": bar_state,
        },
        default_outcome="defaulted",
        outcome_map={
            "outcome1": {
                "FOO": "outcome1",
                "BAR": "outcome3",
            },
            "outcome2": {
                "FOO": "outcome2",
                "BAR": "outcome3",
            },
        },
    )
    concurrence_state.set_description(
        "Executes FooState and BarState in parallel and maps their combined outcomes to the next transition."
    )
    concurrence_state.add_input_key(
        "foo_str",
        "String read by BarState during the concurrent execution.",
    )
    concurrence_state.add_output_key(
        "foo_str",
        "String written by FooState during the concurrent execution.",
    )

    # Add concurrent state to the FSM
    sm.add_state(
        "CONCURRENCE",
        concurrence_state,
        transitions={
            "outcome1": "CONCURRENCE",
            "outcome2": "CONCURRENCE",
            "defaulted": "outcome4",
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub(sm, "YASMIN_CONCURRENCE_DEMO")

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
