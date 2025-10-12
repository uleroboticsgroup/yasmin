# Copyright (C) 2025 Georgia Tech Research Institute
# Supported by USDA-NIFA CSIAPP Grant. No. 2023-70442-39232
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

from typing import Dict, Optional
from threading import Thread, Lock

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class Concurrence(State):
    """
    Runs a series of states in parallel

    The Concurrence class runs a set of states concurrently, waiting
    for the termination of each, and then returns a single output
    according to a provided rule map, or a default outcome if no rule is
    satisfied.

    Attributes:
        _states (Dict[str, State]): The states to run concurrently (name -> state).
        _default_outcome (str): Default outcome.
        _outcome_map (Dict[str, Dict[str, str]]): Specifies which combination of state outputs should produce a given
                                                  overall output.
        _intermediate_outcomes (Dict[str, Optional[str]]): Stores the intermediate outcomes of the concurrent states.
        _mutex (Lock): Mutex for intermediate outcome map.
    """

    def __init__(
        self,
        states: Dict[str, State],
        default_outcome: str,
        outcome_map: Dict[str, Dict[State, str]] = {},
    ) -> None:
        """
        Constructs a State with a set of possible outcomes.

        Args:
            states (Dict[str, State]): A map of state names to states that will run concurrently.
            default_outcome (str): The default outcome to return if no outcome map
            rules are satisfied.
            outcome_map (Dict[str, Dict[str, str]]): A map of outcome names to requirements for achieving
            that outcome.

        Raises:
            ValueError: If either the provided outcomes set or states set are empty.
            ValueError: If the same instance of a state is listed to run concurrently with itself.
            KeyError: If an intermediate outcome is not registered with the correlated state.
        """

        if len(outcome_map.keys()) > 0:
            outcomes = list(set(outcome_map.keys()) | set([default_outcome]))
        else:
            outcomes = [default_outcome]

        super().__init__(outcomes)

        if not states:
            raise ValueError("There must be at least one state")

        # Check for duplicate state instances
        state_instances = list(states.values())
        unique_instances = set(id(state) for state in state_instances)
        if len(unique_instances) != len(state_instances):
            raise ValueError("There are duplicate state names in the states")

        self._states: Dict[str, State] = states
        self._default_outcome: str = default_outcome
        self._outcome_map: Dict[str, Dict[str, str]] = {}

        for outcome, requirements in outcome_map.items():
            self._outcome_map[outcome] = {}
            for state_name, state_outcome in requirements.items():

                if state_name not in self._states:
                    raise KeyError(
                        f"State name '{state_name}' not found in states dictionary"
                    )

                state = self._states[state_name]

                # Check if intermediate outcome belongs to state
                if state_outcome not in state.get_outcomes():
                    raise KeyError(
                        f"Outcome '{state_outcome}' is not registered in state {state_name}"
                    )

                self._outcome_map[outcome][state_name] = state_outcome

        self._intermediate_outcomes: Dict[str, Optional[str]] = {}
        for state_name in self._states.keys():
            self._intermediate_outcomes[state_name] = None

        self._mutex = Lock()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the state's specific logic.

        Args:
            blackboard (Blackboard): A shared pointer to the Blackboard to use during
            execution.

        Returns:
            str: A string representing the outcome of the execution.
        """

        state_threads = []

        for state_name, state in self._states.items():
            state_threads.append(
                Thread(
                    target=self.execute_and_save_state,
                    args=(
                        state,
                        state_name,
                        blackboard,
                    ),
                )
            )
            state_threads[-1].start()

        for t in state_threads:
            t.join()

        satisfied_outcomes = []
        for _, (outcome, requirements) in enumerate(self._outcome_map.items()):
            satisfied = True

            for state_name, state_outcome in requirements.items():
                satisfied = satisfied and (
                    self._intermediate_outcomes[state_name] == state_outcome
                )

            if satisfied:
                satisfied_outcomes.append(outcome)

        if len(satisfied_outcomes) == 0:
            return self._default_outcome

        if len(satisfied_outcomes) > 1:
            yasmin.YASMIN_LOG_WARN(
                f"More than one satisfied outcome after concurrent state execution."
            )

        return satisfied_outcomes[0]

    def execute_and_save_state(
        self, state: State, state_name: str, blackboard: Blackboard
    ) -> None:
        """
        Executes a state and saves its outcome to the intermediate map.

        Args:
            state (State): A state to execute.
            state_name (str): The name of the state to execute.
            blackboard (Blackboard): An instance of Blackboard that provides the context for execution.
        """
        outcome = state(blackboard)
        with self._mutex:
            self._intermediate_outcomes[state_name] = outcome

    def get_states(self) -> Dict[str, State]:
        """
        Returns the map of states managed by this concurrence state.

        Returns:
            Dict[str, State]: A map of state names to states.
        """
        return self._states

    def get_outcome_map(self) -> Dict[str, Dict[str, str]]:
        """
        Returns the outcome map for this concurrence state.

        Returns:
            Dict[str, Dict[str, str]]: A map of outcome names to their requirements.
        """
        return self._outcome_map

    def get_default_outcome(self) -> str:
        """
        Returns the default outcome for this concurrence state.

        Returns:
            str: The default outcome as a string.
        """
        return self._default_outcome

    def cancel_state(self) -> None:
        """
        Cancels the current state execution.

        This method sets the canceled flag to true and logs the action.
        """
        for key in self._states:
            self._states[key].cancel_state()

        super().cancel_state()

    def __str__(self) -> str:
        """
        Converts the state to a string representation.

        Returns:
            str: A string representation of the state.
        """
        result = "Concurrence ["

        for i, key in enumerate(self._states):
            result += f"{key} ({self._states[key]})"

            if i < len(self._states) - 1:
                result += ", "

        result += "]"
        return result
