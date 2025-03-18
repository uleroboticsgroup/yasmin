#!/usr/bin/env python
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
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
    bb = Blackboard()
    bb["msg1"] = "teste1"
    bb["msg2"] = "teste2"

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

    sm.execute(bb)
