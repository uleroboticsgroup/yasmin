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
    Child class of a State that runs multiple other states in parallel threads.

    Attributes:
        _states (Dict[str, State]): A dictionary of states that will be run in parallel by this state.
        _outcome_map (Dict[str, Dict[str, str]]): A dictionary correlating the outcomes of the concurrent states to
                                                  outcomes of this state.
        _default_outcome (str): A default outcome in case none of the correlations in _outcome_map are satisfied.
        _intermediate_outcomes (Dict[str, Optional[str]]): A temporary storage of the parallel states's outcomes.
        _mutex (Lock): A mutex to ensure thread safety of _intermediate_outcomes.
    """

    def __init__(
        self,
        states: Dict[str, State],
        default_outcome: str,
        outcome_map: Dict[str, Dict[State, str]] = {},
    ) -> None:
        """
        Initializes the Concurrence instance.

        :param default_outcome: A default outcome in case none of the correlations in outcome_map are satisfied.
        :param outcome_map: A dictionary correlating the outcomes of the concurrent states to outcomes of this state.
        :param states: A dictionary of states that will be run in parallel by this state.

        :raises ValueError: If either the provided outcomes set or states set are empty.
        :raises ValueError: If the same instance of a state is listed to run concurrently with itself.
        :raises KeyError: If an intermediate outcome is not registered with the correlated state.
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
        Executes the parallel behavior.

        :param blackboard: An instance of Blackboard that provides the context for execution.
        :return: The outcome of the execution as a string.
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

        :param state: A state to execute.
        :param state_name: The name of the state to execute.
        :param blackboard: An instance of Blackboard that provides the context for execution.
        :return: None
        """
        outcome = state(blackboard)
        with self._mutex:
            self._intermediate_outcomes[state_name] = outcome

    def get_states(self) -> Dict[str, State]:
        """
        Returns the dictionary of states managed by this concurrence state.

        :return: A dictionary of states.
        """
        return self._states

    def cancel_state(self) -> None:
        """
        Cancels the execution of all states.
        """
        for key in self._states:
            self._states[key].cancel_state()

        super().cancel_state()

    def __str__(self) -> str:
        """
        Returns a string representation of the concurrence state, listing all states.

        Returns:
            str: A string representation of the state machine.
        """
        result = "Concurrence ["

        for i, key in enumerate(self._states):
            result += f"{key} ({self._states[key]})"

            if i < len(self._states) - 1:
                result += ", "

        result += "]"
        return result
