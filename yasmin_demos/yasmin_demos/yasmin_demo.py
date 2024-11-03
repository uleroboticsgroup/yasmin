#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta
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
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
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

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
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


# Main function to initialize and run the state machine
def main():
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

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
    YasminViewerPub("yasmin_demo", sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
