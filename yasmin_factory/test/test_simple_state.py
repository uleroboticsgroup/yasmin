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


class TestParameterState(State):
    """Test state that accepts parameters."""

    def __init__(self, param1: str, param2: str):
        super().__init__(["success", "failure"])
        self.param1 = param1
        self.param2 = param2

    def execute(self, blackboard: Blackboard) -> str:
        blackboard["param1"] = self.param1
        blackboard["param2"] = self.param2
        return "success"
