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

"""Python bindings for yasmin::CbState"""

from typing import Callable, List, Set, overload
from yasmin.state import State
from yasmin.blackboard import Blackboard

class CbState(State):
    """
    A state that executes a callback function.

    CbState is a convenience state that wraps a Python callback function,
    allowing for quick creation of simple states without defining a new class.
    """

    @overload
    def __init__(self, outcomes: Set[str], callback: Callable[[Blackboard], str]) -> None:
        """
        Construct a CbState with outcomes as a set and a callback function.

        Args:
            outcomes: Set of possible outcome strings
            callback: Function that takes a Blackboard and returns an outcome string
        """
        ...

    @overload
    def __init__(
        self, outcomes: List[str], callback: Callable[[Blackboard], str]
    ) -> None:
        """
        Construct a CbState with outcomes as a list and a callback function.

        Args:
            outcomes: List of possible outcome strings
            callback: Function that takes a Blackboard and returns an outcome string
        """
        ...

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the callback function with the provided blackboard.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string returned by the callback
        """
        ...

    def to_string(self) -> str:
        """
        Convert the CbState to a string representation.

        Returns:
            String representation of the CbState
        """
        ...

    def __str__(self) -> str:
        """
        Convert the CbState to a string representation.

        Returns:
            String representation of the CbState
        """
        ...

    def __call__(self, blackboard: Blackboard) -> str:
        """
        Execute the CbState by calling it like a function.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string returned by the callback
        """
        ...
