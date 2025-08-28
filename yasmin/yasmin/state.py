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
    Abstract base class representing a state in a state machine.

    This class provides a framework for creating specific states with defined outcomes.
    Subclasses must implement the `execute` method to define the state-specific behavior.

    Attributes:
        _outcomes (Set[str]): A set of valid outcomes for this state.
        __status (StateStatus): Current status of the state.
        __status_lock (threading.Lock): Lock for thread-safe status operations.
    """

    def __init__(self, outcomes: Set[str]) -> None:
        """
        Initializes the State instance.

        :param outcomes: A set of valid outcomes for this state.
                         Must contain at least one outcome.
        :raises ValueError: If the provided outcomes set is empty.
        """

        ## A set of valid outcomes for this state.
        self._outcomes: Set = set()
        ## Current status of the state
        self.__status: StateStatus = StateStatus.IDLE
        ## Lock for thread-safe status operations
        self.__status_lock: Lock = Lock()

        if outcomes:
            self._outcomes.update(outcomes)
            self._outcomes: Set = sorted(self._outcomes)
        else:
            raise ValueError("There must be at least one outcome")

    def get_status(self) -> StateStatus:
        """
        Gets the current status of the state.

        :return: The current StateStatus.
        """
        with self.__status_lock:
            return self.__status

    def set_status(self, status: StateStatus) -> None:
        """
        Sets the current status of the state.

        :param status: The StateStatus to set.
        """
        with self.__status_lock:
            self.__status = status

    def is_idle(self) -> bool:
        """
        Checks if the state is idle.

        :return: True if the state is idle, False otherwise.
        """
        return self.get_status() == StateStatus.IDLE

    def is_running(self) -> bool:
        """
        Checks if the state is currently running.

        :return: True if the state is running, False otherwise.
        """
        return self.get_status() == StateStatus.RUNNING

    def is_canceled(self) -> bool:
        """
        Checks if the state has been canceled.

        :return: True if the state is canceled, False otherwise.
        """
        return self.get_status() == StateStatus.CANCELED

    def is_completed(self) -> bool:
        """
        Checks if the state has completed execution.

        :return: True if the state is completed, False otherwise.
        """
        return self.get_status() == StateStatus.COMPLETED

    def __call__(self, blackboard: Blackboard = None) -> str:
        """
        Executes the state and returns the outcome.

        This method sets the state as running, executes the state's behavior,
        and validates the outcome against the allowed outcomes.

        :param blackboard: An optional Blackboard instance that can be used during execution.
                           If None, a new Blackboard instance will be created.
        :return: The outcome of the state execution.
        :raises ValueError: If the outcome is not one of the valid outcomes for this state.
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
        Executes the specific behavior of the state.

        This method must be implemented by subclasses to define what happens
        when the state is executed.

        :param blackboard: An instance of Blackboard that provides the context for execution.
        :return: The outcome of the execution as a string.
        :raises NotImplementedError: If not implemented in a subclass.
        """
        raise NotImplementedError("Subclasses must implement the execute method")

    def cancel_state(self) -> None:
        """
        Cancels the execution of the state.

        Sets the status to CANCELED and logs the cancellation.
        """
        yasmin.YASMIN_LOG_INFO(f"Canceling state '{self}'")
        self.set_status(StateStatus.CANCELED)

    def get_outcomes(self) -> Set[str]:
        """
        Gets the valid outcomes for this state.

        :return: A set of valid outcomes as strings.
        """
        return self._outcomes

    def __str__(self) -> str:
        """
        Returns the string representation of the state.

        :return: The name of the class representing the state.
        """
        return self.__class__.__name__
