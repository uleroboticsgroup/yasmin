# Copyright (C) 2025 Pedro Edom Nunes
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

import yasmin
from yasmin import Blackboard, State


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
        self.log_prefix = "Observed value"
        self.sleep_ms = 300
        self.set_description(
            "Prints the value stored in 'foo_str' from the blackboard and transitions back to the Foo state."
        )
        self.set_outcome_description("outcome3", "Final outcome")
        self.declare_parameter(
            "log_prefix",
            "Prefix printed before the blackboard value.",
            "Observed value",
        )
        self.declare_parameter(
            "sleep_ms",
            "Delay in milliseconds before each execution.",
            300,
        )
        self.add_input_key(
            "foo_str",
            "String produced by FooState and printed by this state.",
        )

    def configure(self) -> None:
        self.log_prefix = self.get_parameter("log_prefix")
        self.sleep_ms = self.get_parameter("sleep_ms")

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
        time.sleep(self.sleep_ms / 1000.0)

        yasmin.YASMIN_LOG_INFO(f"{self.log_prefix}: {blackboard['foo_str']}")
        return "outcome3"
