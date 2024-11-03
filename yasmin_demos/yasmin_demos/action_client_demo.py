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

import rclpy
from action_tutorials_interfaces.action import Fibonacci
import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


class FibonacciState(ActionState):
    """
    Class representing the state of the Fibonacci action.

    Inherits from ActionState and implements methods to handle the
    Fibonacci action in a finite state machine.

    Attributes:
        None
    """

    def __init__(self) -> None:
        """
        Initializes the FibonacciState.

        Sets up the action type and the action name for the Fibonacci
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            Fibonacci,  # action type
            "/fibonacci",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:
        """
        Creates the goal for the Fibonacci action.

        This method retrieves the input value from the blackboard and
        populates the Fibonacci goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Fibonacci.Goal: The populated goal object for the Fibonacci action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = Fibonacci.Goal()
        goal.order = blackboard["n"]  # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: Fibonacci.Result) -> str:
        """
        Handles the response from the Fibonacci action.

        This method processes the result of the Fibonacci action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Fibonacci.Result): The result object from the Fibonacci action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["fibo_res"] = (
            response.sequence
        )  # Store the result sequence in the blackboard
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: Fibonacci.Feedback
    ) -> None:
        """
        Prints feedback from the Fibonacci action.

        This method logs the partial sequence received during the action.

        Parameters:
            blackboard (Blackboard): The blackboard (not used in this method).
            feedback (Fibonacci.Feedback): The feedback object from the Fibonacci action.

        Returns:
            None

        Raises:
            None
        """
        yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.partial_sequence)}")


def print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Fibonacci action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['fibo_res']}")
    return SUCCEED


def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "CALLING_FIBONACCI",
        FibonacciState(),
        transitions={
            SUCCEED: "PRINTING_RESULT",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "PRINTING_RESULT",
        CbState([SUCCEED], print_result),
        transitions={
            SUCCEED: "outcome4",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["n"] = 10  # Set the Fibonacci order to 10

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
