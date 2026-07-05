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


class FooState(State):
    """
    Represents the Foo state in the state machine.
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
        self.counter_prefix = "Counter"
        self.max_count = 3
        self.sleep_ms = 300
        self.set_description(
            "Produces a counter string and stores it in the blackboard while the counter is below the threshold."
        )
        self.set_outcome_description("outcome1", "Counter is below the threshold")
        self.set_outcome_description("outcome2", "Counter reached the threshold")
        self.declare_parameter(
            "counter_prefix",
            "Prefix used when formatting the counter string.",
            "Counter",
        )
        self.declare_parameter(
            "max_count",
            "Number of successful loops before the state returns outcome2.",
            3,
        )
        self.declare_parameter(
            "sleep_ms",
            "Delay in milliseconds before each execution.",
            300,
        )
        self.add_output_key(
            "foo_str",
            "String containing the current counter value produced by FooState.",
        )

    def configure(self) -> None:
        self.counter_prefix = self.get_parameter("counter_prefix")
        self.max_count = self.get_parameter("max_count")
        self.sleep_ms = self.get_parameter("sleep_ms")

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
        time.sleep(self.sleep_ms / 1000.0)

        if self.counter < self.max_count:
            self.counter += 1
            blackboard["foo_str"] = f"{self.counter_prefix}: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
