#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta
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
from example_interfaces.srv import AddTwoInts
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

import yasmin
from yasmin import Blackboard, CbState, StateMachine
from yasmin_ros import ServiceState, set_ros_loggers
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    """
    A state that calls the AddTwoInts service to add two integers.

    This class is a state in a finite state machine that sends a request
    to the AddTwoInts service, retrieves the response, and updates the
    blackboard with the result.
    """

    def __init__(self) -> None:
        """
        Initializes the AddTwoIntsState.

        Calls the parent constructor with the specific service type,
        service name, request handler, outcomes, and response handler.
        """
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler,  # cb to process the response
        )
        self.set_description(
            "Calls the AddTwoInts service using the values stored in the blackboard and writes the resulting sum back to the blackboard."
        )
        self.add_input_key(
            "a", description="First integer used for the service request."
        )
        self.add_input_key(
            "b", description="Second integer used for the service request."
        )
        self.add_output_key(
            "sum", description="Sum returned by the AddTwoInts service."
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:
        """
        Creates the service request from the blackboard data.

        Args:
            blackboard (Blackboard): The blackboard containing the input values.

        Returns:
            AddTwoInts.Request: The request object populated with values from the blackboard.
        """
        req = AddTwoInts.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self, blackboard: Blackboard, response: AddTwoInts.Response
    ) -> str:
        """
        Processes the response from the AddTwoInts service.

        Updates the blackboard with the sum result from the response.

        Args:
            blackboard (Blackboard): The blackboard to update with the sum.
            response (AddTwoInts.Response): The response from the service call.

        Returns:
            str: The outcome of the operation, which is "outcome1".
        """
        blackboard["sum"] = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    """
    Sets the integer values in the blackboard.

    This function initializes the blackboard with two integer values to be added.

    Args:
        blackboard (Blackboard): The blackboard to update with integer values.

    Returns:
        str: The outcome of the operation, which is SUCCEED.
    """
    blackboard["a"] = 10
    blackboard["b"] = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    """
    Logs the sum value from the blackboard.

    This function retrieves the sum from the blackboard and logs it.

    Args:
        blackboard (Blackboard): The blackboard from which to retrieve the sum.

    Returns:
        str: The outcome of the operation, which is SUCCEED.
    """
    yasmin.YASMIN_LOG_INFO(f"Sum: {blackboard['sum']}")
    return SUCCEED


def main() -> None:
    # Init ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_service_client_demo")

    # Create a FSM
    sm = StateMachine(outcomes=["outcome4"], handle_sigint=True)
    sm.set_description(
        "Sets two integers in the blackboard, calls the AddTwoInts service, and prints the resulting sum."
    )
    sm.add_output_key("a", description="First integer used for the service request.")
    sm.add_output_key("b", description="Second integer used for the service request.")
    sm.add_output_key("sum", description="Sum returned by the AddTwoInts service.")

    setting_ints_state = CbState([SUCCEED], set_ints)
    setting_ints_state.set_description(
        "Writes the two input integers for the AddTwoInts service into the blackboard."
    )
    setting_ints_state.add_output_key(
        "a",
        description="First integer used for the service request.",
    )
    setting_ints_state.add_output_key(
        "b",
        description="Second integer used for the service request.",
    )

    printing_sum_state = CbState([SUCCEED], print_sum)
    printing_sum_state.set_description(
        "Reads the computed sum from the blackboard and logs it."
    )
    printing_sum_state.add_input_key(
        "sum",
        description="Sum previously written to the blackboard by the service state.",
    )

    # Add states
    sm.add_state(
        "SETTING_INTS",
        setting_ints_state,
        transitions={SUCCEED: "ADD_TWO_INTS"},
    )
    sm.add_state(
        "ADD_TWO_INTS",
        AddTwoIntsState(),
        transitions={
            "outcome1": "PRINTING_SUM",
            SUCCEED: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "PRINTING_SUM",
        printing_sum_state,
        transitions={
            SUCCEED: "outcome4",
        },
    )

    # Publish FSM info
    YasminViewerPub(sm, "YASMIN_SERVICE_CLIENT_DEMO")

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
