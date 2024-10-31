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


from typing import Set, List, Dict, Any
from typing import Union, Callable
from threading import Lock

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class StateMachine(State):
    def __init__(self, outcomes: Set[str]) -> None:

        super().__init__(outcomes)

        self._states = {}
        self._start_state = None
        self.__current_state = None
        self.__current_state_lock = Lock()
        self.__start_cbs = []
        self.__transition_cbs = []
        self.__end_cbs = []

    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = None,
    ) -> None:

        if not transitions:
            transitions = {}

        if name in self._states:
            raise KeyError(f"State '{name}' already registered in the state machine")

        for key in transitions:
            if not key:
                raise ValueError(f"Transitions with empty source in state '{name}'")

            if not transitions[key]:
                raise ValueError(f"Transitions with empty target in state '{name}'")

            if key not in state.get_outcomes():
                raise KeyError(
                    f"State '{name}' references unregistered outcomes '{key}', available outcomes are {state.get_outcomes()}"
                )

        transition_string = ""
        for key in transitions:
            transition_string += "\n\t" + key + " --> " + transitions[key]

        yasmin.YASMIN_LOG_DEBUG(
            f"Adding state '{name}' of type '{state}' with transitions: {transition_string}"
        )

        self._states[name] = {"state": state, "transitions": transitions}

        if not self._start_state:
            self.set_start_state(name)

    def set_start_state(self, state_name: str) -> None:

        if not state_name:
            raise ValueError("Initial state cannot be empty")

        elif state_name not in self._states:
            raise KeyError(f"Initial state '{state_name}' is not in the state machine")

        yasmin.YASMIN_LOG_DEBUG(f"Setting start state to '{state_name}'")

        self._start_state = state_name

    def get_start_state(self) -> str:
        return self._start_state

    def get_states(self) -> Dict[str, Union[State, Dict[str, str]]]:
        return self._states

    def get_current_state(self) -> str:
        with self.__current_state_lock:
            if self.__current_state:
                return self.__current_state

        return ""

    def add_start_cb(self, cb: Callable, args: List[Any] = None) -> None:
        if args is None:
            args = []
        self.__start_cbs.append((cb, args))

    def add_transition_cb(self, cb: Callable, args: List[Any] = None) -> None:
        if args is None:
            args = []
        self.__transition_cbs.append((cb, args))

    def add_end_cb(self, cb: Callable, args: List[Any] = None) -> None:
        if args is None:
            args = []
        self.__end_cbs.append((cb, args))

    def _call_start_cbs(self, blackboard: Blackboard, start_state: str) -> None:
        try:
            for cb, args in self.__start_cbs:
                cb(blackboard, start_state, *args)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not execute start callback: {e}")

    def _call_transition_cbs(
        self,
        blackboard: Blackboard,
        from_state: str,
        to_state: str,
        outcome: str,
    ) -> None:
        try:
            for cb, args in self.__transition_cbs:
                cb(blackboard, from_state, to_state, outcome, *args)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not execute transition callback: {e}")

    def _call_end_cbs(self, blackboard: Blackboard, outcome: str) -> None:
        try:
            for cb, args in self.__end_cbs:
                cb(blackboard, outcome, *args)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not execute end callback: {e}")

    def validate(self) -> None:

        yasmin.YASMIN_LOG_DEBUG("Validating state machine")

        # check initial state
        if not self._start_state:
            raise RuntimeError("No initial state set")

        terminal_outcomes = []

        # check all states
        for state_name in self._states:

            state: State = self._states[state_name]["state"]
            transitions: Dict[str, str] = self._states[state_name]["transitions"]

            outcomes = state.get_outcomes()

            # check if all state outcomes are in transitions
            for o in outcomes:
                if o not in set(list(transitions.keys()) + self.get_outcomes()):
                    raise KeyError(
                        f"State '{state_name}' outcome '{o}' not registered in transitions"
                    )

                # state outcomes that are in state machines out do not need transitions
                elif o in self.get_outcomes():
                    terminal_outcomes.append(o)

            # if sate is a state machine, validate it
            if isinstance(state, StateMachine):
                state.validate()

            # add terminal outcomes
            terminal_outcomes.extend([transitions[key] for key in transitions])

        # check terminal outcomes for the state machine
        terminal_outcomes = set(terminal_outcomes)

        # check if all state machine outcomes are in the terminal outcomes
        for o in self.get_outcomes():
            if o not in terminal_outcomes:
                raise KeyError(f"Target outcome '{o}' not registered in transitions")

        # check if all terminal outcomes are states or state machine outcomes
        for o in terminal_outcomes:
            if o not in set(list(self._states.keys()) + self.get_outcomes()):
                raise KeyError(
                    f"State machine outcome '{o}' not registered as outcome neither state"
                )

    def execute(self, blackboard: Blackboard) -> str:

        self.validate()

        yasmin.YASMIN_LOG_INFO(
            f"Executing state machine with initial state '{self._start_state}'"
        )
        self._call_start_cbs(blackboard, self._start_state)

        with self.__current_state_lock:
            self.__current_state = self._start_state

        while not self.is_canceled():

            with self.__current_state_lock:
                state = self._states[self.__current_state]

            outcome = state["state"](blackboard)
            old_outcome = outcome

            # check outcome belongs to state
            if outcome not in state["state"].get_outcomes():
                raise KeyError(
                    f"Outcome '{outcome}' is not register in state {self.__current_state}"
                )

            # translate outcome using transitions
            if outcome in state["transitions"]:
                outcome = state["transitions"][outcome]

            # outcome is an outcome of the sm
            if outcome in self.get_outcomes():
                with self.__current_state_lock:
                    self.__current_state = None

                yasmin.YASMIN_LOG_INFO(f"State machine ends with outcome '{outcome}'")
                self._call_end_cbs(blackboard, outcome)
                return outcome

            # outcome is a state
            elif outcome in self._states:
                yasmin.YASMIN_LOG_INFO(
                    f"State machine transitioning '{self.__current_state}' : '{old_outcome}' --> '{outcome}'"
                )

                self._call_transition_cbs(
                    blackboard,
                    self.get_current_state(),
                    outcome,
                    old_outcome,
                )

                with self.__current_state_lock:
                    self.__current_state = outcome

            # outcome is not in the sm
            else:
                raise KeyError(
                    f"Outcome '{outcome}' is not a state neither a state machine outcome"
                )

    def cancel_state(self) -> None:
        super().cancel_state()
        with self.__current_state_lock:
            if self.__current_state:
                self._states[self.__current_state]["state"].cancel_state()

    def __str__(self) -> str:
        result = "State Machine\n"

        for key in self._states:
            result += f"{key} ({self._states[key]['state']})\n"
            for tkey in self._states[key]["transitions"]:
                result += f"\t{tkey} --> {self._states[key]['transitions'][tkey]}\n"

        return result
