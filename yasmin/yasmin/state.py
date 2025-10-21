# Copyright (C) 2023 Miguel Ángel González Santamarta
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

from enum import Enum
from typing import Set
from threading import Lock
from abc import ABC, abstractmethod

import yasmin
from yasmin.blackboard import Blackboard


class StateStatus(Enum):
    """
    Enumeration representing the current status of a state.
    """

    IDLE = "idle"  # State is idle and ready to execute
    RUNNING = "running"  # State is currently executing
    CANCELED = "canceled"  # State execution has been canceled
    COMPLETED = "completed"  # State execution has completed successfully


class State(ABC):
    """
    Represents a state in a state machine.

    The State class defines a state that can execute actions and manage
    outcomes. It maintains information about its execution status
    and the possible outcomes of its execution.

    Attributes:
        _outcomes (Set[str]): The possible outcomes of this state.
        __status (StateStatus): Current status of the state.
        __status_lock (threading.Lock): Lock for thread-safe status operations.
    """

    def __init__(self, outcomes: Set[str]) -> None:
        """
        Constructs a State with a set of possible outcomes.

        Args:
            outcomes (Set[str]): A set of possible outcomes for this state.

        Raises:
            ValueError: If the provided outcomes set is empty.
        """

        ## The possible outcomes of this state.
        self._outcomes: Set = set()
        ## Current status of the state
        self.__status: StateStatus = StateStatus.IDLE
        ## Lock for thread-safe status operations
        self.__status_lock: Lock = Lock()

        if outcomes:
            self._outcomes.update(outcomes)
            self._outcomes: Set = sorted(self._outcomes)
        else:
            raise ValueError("A state must have at least one possible outcome.")

    def get_status(self) -> StateStatus:
        """
        Gets the current status of the State.

        Returns:
            StateStatus: The current status of the state.
        """
        with self.__status_lock:
            return self.__status

    def set_status(self, status: StateStatus) -> None:
        """
        Sets the status of the State.

        Args:
            status (StateStatus): The new status for the state.
        """
        with self.__status_lock:
            self.__status = status

    def is_idle(self) -> bool:
        """
        Checks if the State is idle.

        Returns:
            bool: True if the state is idle, otherwise False.
        """
        return self.get_status() == StateStatus.IDLE

    def is_running(self) -> bool:
        """
        Checks if the State is running.

        Returns:
            bool: True if the state is running, otherwise False.
        """
        return self.get_status() == StateStatus.RUNNING

    def is_canceled(self) -> bool:
        """
        Checks if the State is canceled.

        Returns:
            bool: True if the state is canceled, otherwise False.
        """
        return self.get_status() == StateStatus.CANCELED

    def is_completed(self) -> bool:
        """
        Checks if the State is completed.

        Returns:
            bool: True if the state is completed, otherwise False.
        """
        return self.get_status() == StateStatus.COMPLETED

    def __call__(self, blackboard: Blackboard = None) -> str:
        """
        Calls the state, transitioning from idle to running, executing, and then to completed.

        Args:
            blackboard (Blackboard, optional): An optional blackboard to share data with the state.

        Returns:
            str: The outcome of the state execution.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Executing state '{self}'")

        self.set_status(StateStatus.RUNNING)

        if blackboard is None:
            blackboard = Blackboard()

        outcome = self.execute(blackboard)

        if outcome not in self._outcomes:
            self.set_status(StateStatus.IDLE)
            raise ValueError(
                f"Outcome '{outcome}' does not belong to the outcomes of the state '{self}'. "
                f"The possible outcomes are: {self._outcomes}"
            )

        # Mark as completed if not canceled
        if self.get_status() != StateStatus.CANCELED:
            self.set_status(StateStatus.COMPLETED)

        return outcome

    @abstractmethod
    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the state's specific logic.

        This method is intended to be overridden by derived classes to provide
        specific execution logic.

        Args:
            blackboard (Blackboard): A shared pointer to the Blackboard to use during
            execution.

        Returns:
            str: A string representing the outcome of the execution.
        """
        raise NotImplementedError("Subclasses must implement the execute method")

    def cancel_state(self) -> None:
        """
        Cancels the current state execution.

        This method sets the status to CANCELED and logs the action.
        """
        yasmin.YASMIN_LOG_INFO(f"Canceling state '{self}'")
        self.set_status(StateStatus.CANCELED)

    def get_outcomes(self) -> Set[str]:
        """
        Gets the set of possible outcomes for this state.

        Returns:
            Set[str]: A constant reference to the set of possible outcomes.
        """
        return self._outcomes

    def __str__(self) -> str:
        """
        Converts the state to a string representation.

        Returns:
            str: A string representation of the state.
        """
        return self.__class__.__name__
