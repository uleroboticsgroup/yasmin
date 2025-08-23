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

import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED


class Foo(State):
    """
    Represents the Foo state in the state machine.
    """

    def __init__(self):
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            SUCCEED: Indicates the state should continue to the next state.
        """
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be SUCCEED.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        data = blackboard["foo_data"]
        yasmin.YASMIN_LOG_INFO(f"{data}")
        blackboard["foo_out_data"] = data
        return SUCCEED


class BarState(State):
    """
    Represents the Bar state in the state machine.

    """

    def __init__(self):
        """
        Initializes the BarState instance, setting up the outcomes.

        Outcomes:
            SUCCEDED: Indicates the state should continue to the next state.
        """
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be SUCCEED.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        data = blackboard["bar_data"]
        yasmin.YASMIN_LOG_INFO(f"{data}")
        return SUCCEED


if __name__ == "__main__":
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """

    rclpy.init()
    set_ros_loggers()

    bb = Blackboard()
    bb["msg1"] = "test1"
    bb["msg2"] = "test2"

    sm = StateMachine(outcomes=[SUCCEED])
    sm.add_state(
        "STATE1",
        Foo(),
        transitions={SUCCEED: "STATE2"},
        remappings={"foo_data": "msg1"},
    )
    sm.add_state(
        "STATE2",
        Foo(),
        transitions={SUCCEED: "STATE3"},
        remappings={"foo_data": "msg2"},
    )
    sm.add_state(
        "STATE3",
        BarState(),
        transitions={SUCCEED: SUCCEED},
        remappings={"bar_data": "foo_out_data"},
    )

    # Execute the FSM
    try:
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()
