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
from action_tutorials_interfaces.action import Fibonacci
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


class FibonacciState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Fibonacci,  # action type
            "/fibonacci",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # cb to process the response
            self.print_feedback,  # cb to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:

        goal = Fibonacci.Goal()
        goal.order = blackboard["n"]
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: Fibonacci.Result
    ) -> str:

        blackboard["fibo_res"] = response.sequence
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: Fibonacci.Feedback
    ) -> None:
        print(f"Received feedback: {list(feedback.partial_sequence)}")


def print_result(blackboard: Blackboard) -> str:
    print(f"Result: {blackboard['fibo_res']}")
    return SUCCEED


# main
def main():

    print("yasmin_action_client_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "CALLING_FIBONACCI",
        FibonacciState(),
        transitions={SUCCEED: "PRINTING_RESULT", CANCEL: "outcome4", ABORT: "outcome4"},
    )
    sm.add_state(
        "PRINTING_RESULT",
        CbState([SUCCEED], print_result),
        transitions={SUCCEED: "outcome4"},
    )

    # pub FSM info
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # create an initial blackoard
    blackboard = Blackboard()
    blackboard["n"] = 10

    # execute FSM
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
