# Copyright (C) 2023  Miguel Ángel González Santamarta
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

from typing import Set
from abc import ABC, abstractmethod

import yasmin
from yasmin.blackboard import Blackboard


class State(ABC):
    """
    Abstract base class representing a state in a state machine.

    This class provides a framework for creating specific states with defined outcomes.
    Subclasses must implement the `execute` method to define the state-specific behavior.

    Attributes:
        _outcomes (Set[str]): A set of valid outcomes for this state.
        _running (bool): A flag indicating whether the state is currently running.
        _canceled (bool): A flag indicating whether the state has been canceled.
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
        ## A flag indicating whether the state is currently running.
        self._running: bool = False
        ## A flag indicating whether the state has been canceled.
        self._canceled: bool = False

        if outcomes:
            self._outcomes.update(outcomes)
            self._outcomes: Set = sorted(self._outcomes)
        else:
            raise ValueError("There must be at least one outcome")

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

        self._canceled: bool = False
        self._running: bool = True

        if blackboard is None:
            blackboard = Blackboard()

        outcome = self.execute(blackboard)

        if outcome not in self._outcomes:
            raise ValueError(
                f"Outcome '{outcome}' does not belong to the outcomes of the state '{self}'. "
                f"The possible outcomes are: {self._outcomes}"
            )

        self._running: bool = False
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

    def __str__(self) -> str:
        """
        Returns the string representation of the state.

        :return: The name of the class representing the state.
        """
        return self.__class__.__name__

    def cancel_state(self) -> None:
        """
        Cancels the execution of the state.

        Sets the _canceled flag to True and logs the cancellation.
        """
        yasmin.YASMIN_LOG_INFO(f"Canceling state '{self}'")
        self._canceled: bool = True

    def is_canceled(self) -> bool:
        """
        Checks if the state has been canceled.

        :return: True if the state is canceled, False otherwise.
        """
        return self._canceled

    def is_running(self) -> bool:
        """
        Checks if the state is currently running.

        :return: True if the state is running, False otherwise.
        """
        return self._running

    def get_outcomes(self) -> Set[str]:
        """
        Gets the valid outcomes for this state.

        :return: A set of valid outcomes as strings.
        """
        return self._outcomes
