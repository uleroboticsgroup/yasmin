# Copyright (C) 2026 Miguel Ángel González Santamarta
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


class WorkerState(State):
    """
    Reusable state that counts iterations and returns 'working' until
    the configured max_count is reached, then returns 'done'.
    """

    def __init__(self) -> None:
        super().__init__(["working", "done"])
        self.counter = 0
        self.max_count = 3
        self.sleep_ms = 500
        self.set_description(
            "Counts iterations and returns 'working' until max_count is reached."
        )
        self.set_outcome_description("working", "Counter is below the threshold")
        self.set_outcome_description("done", "Counter reached the threshold")
        self.declare_parameter(
            "max_count",
            "Number of iterations before returning 'done'.",
            3,
        )
        self.declare_parameter(
            "sleep_ms",
            "Delay in milliseconds before each execution.",
            500,
        )

    def configure(self) -> None:
        self.max_count = self.get_parameter("max_count")
        self.sleep_ms = self.get_parameter("sleep_ms")
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO(
            f"Executing WorkerState: iteration {self.counter + 1}/{self.max_count}"
        )
        time.sleep(self.sleep_ms / 1000.0)

        self.counter += 1
        if self.counter >= self.max_count:
            return "done"
        return "working"
