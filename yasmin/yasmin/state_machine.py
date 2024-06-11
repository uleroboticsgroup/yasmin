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
from yasmin.state import State
from yasmin.yasmin_logs import YASMIN_LOG_INFO
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
        transitions: Dict[str, str] = None
    ) -> None:

        if not transitions:
            transitions = {}

        self._states[name] = {
            "state": state,
            "transitions": transitions
        }

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

    def execute(self, blackboard: Blackboard) -> str:

        with self.__current_state_lock:
            self.__current_state = self._start_state

        while True:

            with self.__current_state_lock:
                state = self._states[self.__current_state]

            outcome = state["state"](blackboard)

            # check outcome belongs to state
            if outcome not in state["state"].get_outcomes():
                raise Exception(
                    f"Outcome ({outcome}) is not register in state {self.__current_state}")

            # translate outcome using transitions
            if outcome in state["transitions"]:
                YASMIN_LOG_INFO(
                    "%s: %s --> %s",
                    self.__current_state, outcome, state["transitions"][outcome]
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
