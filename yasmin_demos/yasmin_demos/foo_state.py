#!/usr/bin/env python3
# coding: utf-8

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
from yasmin import State
from yasmin import Blackboard


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
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
        time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
