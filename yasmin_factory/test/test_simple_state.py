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
