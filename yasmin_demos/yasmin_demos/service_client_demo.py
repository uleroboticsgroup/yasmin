#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy
from example_interfaces.srv import AddTwoInts
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:

        req = AddTwoInts.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self, blackboard: Blackboard, response: AddTwoInts.Response
    ) -> str:

        blackboard["sum"] = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    blackboard["a"] = 10
    blackboard["b"] = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    print(f"Sum: {blackboard['sum']}")
    return SUCCEED


# main
def main():

    print("yasmin_service_client_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "SETTING_INTS",
        CbState([SUCCEED], set_ints),
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
        "PRINTING_SUM", CbState([SUCCEED], print_sum), transitions={SUCCEED: "outcome4"}
    )

    # pub FSM info
    YasminViewerPub("YASMIN_SERVICE_CLIENT_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
