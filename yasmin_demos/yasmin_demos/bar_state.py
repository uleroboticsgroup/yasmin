# Copyright (C) 2025 Pedro Edom Nunes
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
