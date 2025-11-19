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

"""Python bindings for yasmin::State"""

from enum import Enum
from typing import List, Set, Union, overload
from yasmin.blackboard import Blackboard

class StateStatus(Enum):
    """
    Enumeration representing the status of a state.

    Attributes:
        IDLE: State is idle and not executing
        RUNNING: State is currently executing
        CANCELED: State execution was canceled
        COMPLETED: State execution has completed
    """

    IDLE: int
    RUNNING: int
    CANCELED: int
    COMPLETED: int

class State:
    """
    Base class for all states in the YASMIN framework.

    A State represents a single unit of execution that can be combined into
    state machines and other hierarchical structures. States have a set of
    possible outcomes and execute logic when called.
    """

    @overload
    def __init__(self, outcomes: Set[str]) -> None:
        """
        Initialize a State with a set of outcomes.

        Args:
            outcomes: Set of possible outcome strings
        """
        ...

    @overload
    def __init__(self, outcomes: List[str]) -> None:
        """
        Initialize a State with a list of outcomes.

        Args:
            outcomes: List of possible outcome strings
        """
        ...

    def get_status(self) -> StateStatus:
        """
        Get the current status of the state.

        Returns:
            The current state status
        """
        ...

    def is_idle(self) -> bool:
        """
        Check if the state is idle.

        Returns:
            True if state is idle, False otherwise
        """
        ...

    def is_running(self) -> bool:
        """
        Check if the state is currently running.

        Returns:
            True if state is running, False otherwise
        """
        ...

    def is_canceled(self) -> bool:
        """
        Check if the state has been canceled.

        Returns:
            True if state was canceled, False otherwise
        """
        ...

    def is_completed(self) -> bool:
        """
        Check if the state has completed execution.

        Returns:
            True if state completed, False otherwise
        """
        ...

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the state's specific logic.

        This method should be overridden in subclasses to implement
        the state's behavior.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string indicating the result of execution
        """
        ...

    def cancel_state(self) -> None:
        """
        Cancel the current state execution.

        This method can be overridden to implement custom cancellation logic.
        """
        ...

    def get_outcomes(self) -> Set[str]:
        """
        Get the set of possible outcomes for this state.

        Returns:
            Set of outcome strings
        """
        ...

    def to_string(self) -> str:
        """
        Convert the state to a string representation.

        Returns:
            String representation of the state
        """
        ...

    def __str__(self) -> str:
        """
        Convert the state to a string representation.

        Returns:
            String representation of the state
        """
        ...

    def __call__(self, blackboard: Blackboard) -> str:
        """
        Execute the state by calling it like a function.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string indicating the result of execution
        """
        ...
