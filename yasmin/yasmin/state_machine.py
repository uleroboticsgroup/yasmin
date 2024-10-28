# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import Dict, List, Union
from threading import Lock

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class StateMachine(State):
    def __init__(self, outcomes: List[str]) -> None:

        super().__init__(outcomes)

        self._states = {}
        self._start_state = None
        self.__current_state = None
        self.__current_state_lock = Lock()

    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = None,
    ) -> None:

        if not transitions:
            transitions = {}

        if name in self._states:
            raise Exception(f"State '{name}' already registered in the state machine")

        for key in transitions:
            if not key:
                raise Exception(f"Transitions with empty source in state '{name}'")

            if not transitions[key]:
                raise Exception(f"Transitions with empty target in state '{name}'")

            if key not in state.get_outcomes():
                raise Exception(
                    f"State '{name}' references unregistered outcomes: '{key}', available outcomes are: {state.get_outcomes()}"
                )

        self._states[name] = {"state": state, "transitions": transitions}

        if not self._start_state:
            self._start_state = name

    def set_start_state(self, name: str) -> None:
        self._start_state = name

    def get_start_state(self) -> str:
        return self._start_state

    def cancel_state(self) -> None:
        super().cancel_state()
        with self.__current_state_lock:
            if self.__current_state:
                self._states[self.__current_state]["state"].cancel_state()

    def validate(self, raise_exception: bool = True) -> str:
        errors = ""

        # check initial state
        if self._start_state is None:
            errors += "\n\tNo initial state set."
        elif self._start_state not in self._states:
            errors += f"\n\tInitial state label: '{self._start_state}' is not in the state machine."

        terminal_outcomes = []

        # check all states
        for state_name in self._states:

            state: State = self._states[state_name]["state"]
            transitions: Dict[str, str] = self._states[state_name]["transitions"]

            outcomes = state.get_outcomes()

            # check if all state outcomes are in transitions
            for o in outcomes:
                if o not in set(list(transitions.keys()) + self.get_outcomes()):
                    errors += f"\n\tState '{state_name}' outcome '{o}' not registered in transitions"

                # state outcomes that are in state machines out do not need transitions
                elif o in self.get_outcomes():
                    terminal_outcomes.append(o)

            # if sate is a state machine, validate it
            if isinstance(state, StateMachine):
                aux_errors = state.validate(False)
                if aux_errors:
                    errors += f"\n\tState machine '{state_name}' failed validation check\n{aux_errors}"

            # add terminal outcomes
            terminal_outcomes.extend([transitions[key] for key in transitions])

        # check terminal outcomes for the state machine
        terminal_outcomes = set(terminal_outcomes)

        # check if all state machine outcomes are in the terminal outcomes
        for o in self.get_outcomes():
            if o not in terminal_outcomes:
                errors += f"\n\tTarget outcome '{o}' not registered in transitions"

        # check if all terminal outcomes are states or state machine outcomes
        for o in terminal_outcomes:
            if o not in set(list(self._states.keys()) + self.get_outcomes()):
                errors += f"\n\tState machine outcome '{o}' not registered as outcome neither state"

        if errors:
            errors = f"{'*' * 100}\nState machine failed validation check:{errors}\n\n\tAvailable states: {list(self._states.keys())}\n{'*' * 100}"

            if raise_exception:
                raise Exception(errors)

        return errors

    def execute(self, blackboard: Blackboard) -> str:

        self.validate()

        with self.__current_state_lock:
            self.__current_state = self._start_state

        while True:

            with self.__current_state_lock:
                state = self._states[self.__current_state]

            outcome = state["state"](blackboard)

            # check outcome belongs to state
            if outcome not in state["state"].get_outcomes():
                raise Exception(
                    f"Outcome ({outcome}) is not register in state {self.__current_state}"
                )

            # translate outcome using transitions
            if outcome in state["transitions"]:
                yasmin.YASMIN_LOG_INFO(
                    f"{self.__current_state}: {outcome} --> {state['transitions'][outcome]}"
                )
                outcome = state["transitions"][outcome]

            # outcome is an outcome of the sm
            if outcome in self.get_outcomes():
                with self.__current_state_lock:
                    self.__current_state = None
                return outcome

            # outcome is a state
            elif outcome in self._states:
                with self.__current_state_lock:
                    self.__current_state = outcome

            # outcome is not in the sm
            else:
                raise Exception(f"Outcome ({outcome}) without transition")

    def get_states(self) -> Dict[str, Union[State, Dict[str, str]]]:
        return self._states

    def get_current_state(self) -> str:
        with self.__current_state_lock:
            if self.__current_state:
                return self.__current_state

        return ""

    def __str__(self) -> str:
        return f"StateMachine: {self._states}"
