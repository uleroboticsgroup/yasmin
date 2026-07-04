# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from yasmin import State, Blackboard


class TestSimpleState(State):
    """Simple test state for testing purposes."""

    def __init__(self):
        super().__init__(["outcome1", "outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard.contains("counter"):
            blackboard["counter"] = 0

        value = blackboard.get("counter")
        blackboard["counter"] = value + 1
        return "outcome1" if value < 2 else "outcome2"


class TestRemappingState(State):
    """Test state that reads from and writes to specific blackboard keys."""

    def __init__(self):
        super().__init__(["success", "failure"])

    def execute(self, blackboard: Blackboard) -> str:
        # Read from input_key (can be remapped)
        if blackboard.contains("input_key"):
            value = blackboard.get("input_key")
            # Write to output_key (can also be remapped independently)
            blackboard["output_key"] = f"processed_{value}"
            return "success"
        else:
            return "failure"


class TestParameterizedState(State):
    """Python test state that declares parameters and configures local state."""

    def __init__(self):
        super().__init__(["done"])
        self.sleep_ms = 0
        self.declare_parameter("sleep_ms", "Delay before execution", 10)

    def configure(self) -> None:
        self.sleep_ms = self.get_parameter("sleep_ms")

    def execute(self, blackboard: Blackboard) -> str:
        blackboard["configured_sleep_ms"] = self.sleep_ms
        return "done"
