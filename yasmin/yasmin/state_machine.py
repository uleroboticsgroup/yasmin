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

from typing import Set, List, Tuple, Dict, Any
from typing import Union, Callable
from threading import Lock, Event

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class StateMachine(State):
    """
    Represents a state machine that can manage states and transitions
    between them, including callbacks for different state events.

    Attributes:
        _states (Dict[str, Dict[str, Any]]): A dictionary mapping state names to their corresponding state objects and transitions.
        _start_state (str): The name of the initial state of the state machine.
        __current_state (str): The name of the current state being executed.
        __current_state_lock (Lock): A threading lock to manage access to the current state.
        __current_state_event (Event): An event to signal when the current state changes.
        _validated (bool): A flag indicating whether the state machine has been validated.
        __start_cbs (List[Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]]): A list of callbacks to call when the state machine starts.
        __transition_cbs (List[Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]]): A list of callbacks to call during state transitions.
        __end_cbs (List[Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]]): A list of callbacks to call when the state machine ends.
    """

    def __init__(self, outcomes: Set[str]) -> None:
        """
        Initializes the StateMachine with a set of possible outcomes.

        Parameters:
            outcomes (Set[str]): A set of possible outcomes for the state machine.
        """
        super().__init__(outcomes)

        ## A dictionary mapping state names to their corresponding state objects and transitions.
        self._states: Dict[str, Dict[str, Any]] = {}
        ## The name of the initial state of the state machine.
        self._start_state: str = None
        ## The name of the current state being executed.
        self.__current_state: str = None
        ## A threading lock to manage access to the current state.
        self.__current_state_lock: Lock = Lock()
        ## An event to signal when the current state changes.
        self.__current_state_event: Event = Event()

        ## A flag indicating whether the state machine has been validated.
        self._validated: bool = False

        ## A list of callbacks to call when the state machine starts.
        self.__start_cbs: List[
            Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]
        ] = []
        ## A list of callbacks to call during state transitions.
        self.__transition_cbs: List[
            Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]
        ] = []
        ## A list of callbacks to call when the state machine ends.
        self.__end_cbs: List[
            Tuple[Callable[[Blackboard, str, List[Any]], None], List[Any]]
        ] = []

        ## A dictionary of remappings to set in the blackboard in each transition
        self.__remappings: Dict[str, Dict[str, str]] = {}

    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = None,
        remappings: Dict[str, Dict[str, str]] = None,
    ) -> None:
        """
        Adds a new state to the state machine.

        Parameters:
            name (str): The name of the state to add.
            state (State): The State object to associate with the name.
            transitions (Dict[str, str], optional): A dictionary mapping source outcomes to target states. Defaults to None.

        Raises:
            KeyError: If the state name is already registered or is an outcome.
            ValueError: If transitions contain empty keys or values.
            KeyError: If transitions reference unregistered outcomes.
        """
        if not transitions:
            transitions = {}

        transition_string = "\n\t" + "\n\t".join(
            f"{key} --> {transitions[key]}" for key in transitions
        )

        # Check if state name is already in the state machine
        if name in self._states:
            raise KeyError(f"State '{name}' already registered in the state machine")

        # Check if state name is an outcome of the state machine
        if name in self._outcomes:
            raise KeyError(f"State name '{name}' is already registered as an outcome")

        # Check the transitions
        for key in transitions:
            if not key:
                raise ValueError(f"Transitions with empty source in state '{name}'")

            if not transitions[key]:
                raise ValueError(f"Transitions with empty target in state '{name}'")

            if key not in state.get_outcomes():
                raise KeyError(
                    f"State '{name}' references unregistered outcomes '{key}', available outcomes are {list(state.get_outcomes())}"
                )

        yasmin.YASMIN_LOG_DEBUG(
            f"Adding state '{name}' of type '{state}' with transitions: {transition_string}"
        )

        self._states[name] = {"state": state, "transitions": transitions}

        if remappings != None:
            self.__remappings[name] = remappings

        if not self._start_state:
            self.set_start_state(name)

        ## Mark state machine as no validated
        self._validated = False

    def set_start_state(self, state_name: str) -> None:
        """
        Sets the initial state for the state machine.

        Parameters:
            state_name (str): The name of the initial state to set.

        Raises:
            ValueError: If the state name is empty.
            KeyError: If the state name is not found in the states.
        """
        if not state_name:
            raise ValueError("Initial state cannot be empty")

        elif state_name not in self._states:
            raise KeyError(f"Initial state '{state_name}' is not in the state machine")

        yasmin.YASMIN_LOG_DEBUG(f"Setting start state to '{state_name}'")

        self._start_state: str = state_name

        ## Mark state machine as no validated
        self._validated = False

    def get_start_state(self) -> str:
        """
        Retrieves the name of the current starting state.

        Returns:
            str: The name of the starting state.
        """
        return self._start_state

    def get_states(self) -> Dict[str, Union[State, Dict[str, str]]]:
        """
        Retrieves all states in the state machine.

        Returns:
            Dict[str, Union[State, Dict[str, str]]]: A dictionary of states and their transitions.
        """
        return self._states

    def get_current_state(self) -> str:
        """
        Retrieves the name of the current state being executed.

        Returns:
            str: The name of the current state, or an empty string if none is set.
        """
        with self.__current_state_lock:
            if self.__current_state:
                return self.__current_state

        return ""

    def __set_current_state(self, state_name: str) -> None:
        """
        Sets the current state name.

        Parameters:
            state_name (str): The name of the state to set as the current state.
        """
        with self.__current_state_lock:
            self.__current_state = state_name
            self.__current_state_event.set()

    def add_start_cb(self, cb: Callable, args: List[Any] = None) -> None:
        """
        Adds a callback to be called when the state machine starts.

        Parameters:
            cb (Callable): The callback function to execute.
            args (List[Any], optional): A list of arguments to pass to the callback. Defaults to None.
        """
        if args is None:
            args = []
        self.__start_cbs.append((cb, args))

    def add_transition_cb(self, cb: Callable, args: List[Any] = None) -> None:
        """
        Adds a callback to be called during state transitions.

        Parameters:
            cb (Callable): The callback function to execute.
            args (List[Any], optional): A list of arguments to pass to the callback. Defaults to None.
        """
        if args is None:
            args = []
        self.__transition_cbs.append((cb, args))

    def add_end_cb(self, cb: Callable, args: List[Any] = None) -> None:
        """
        Adds a callback to be called when the state machine ends.

        Parameters:
            cb (Callable): The callback function to execute.
            args (List[Any], optional): A list of arguments to pass to the callback. Defaults to None.
        """
        if args is None:
            args = []
        self.__end_cbs.append((cb, args))

    def _call_start_cbs(self, blackboard: Blackboard, start_state: str) -> None:
        """
        Executes all start callbacks.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for sharing state.
            start_state (str): The name of the state machine's starting state.

        Raises:
            Exception: If an error occurs while executing a callback.
        """
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
        """
        Executes all transition callbacks.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for sharing state.
            from_state (str): The state the transition is coming from.
            to_state (str): The state the transition is going to.
            outcome (str): The outcome of the transition.

        Raises:
            Exception: If an error occurs while executing a callback.
        """
        try:
            for cb, args in self.__transition_cbs:
                cb(blackboard, from_state, to_state, outcome, *args)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not execute transition callback: {e}")

    def _call_end_cbs(self, blackboard: Blackboard, outcome: str) -> None:
        """
        Executes all end callbacks.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for sharing state.
            outcome (str): The outcome of the state machine execution.

        Raises:
            Exception: If an error occurs while executing a callback.
        """
        try:
            for cb, args in self.__end_cbs:
                cb(blackboard, outcome, *args)
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not execute end callback: {e}")

    def validate(self, strict_mode: bool = False) -> None:
        """
        Validates the state machine to ensure all states and transitions are correct.

        Parameters:
            strict_mode (bool): Whether the validation is strict, which means checking if all state outcomes are used and all state machine outcomes are reached.

        Raises:
            RuntimeError: If no initial state is set.
            KeyError: If there are any unregistered outcomes or transitions.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Validating state machine '{self}'")

        if self._validated and not strict_mode:
            yasmin.YASMIN_LOG_DEBUG(f"State machine '{self}' has already been validated")

        # Terminal outcomes from all transitions
        terminal_outcomes = []

        # Check initial state
        if not self._start_state:
            raise RuntimeError("No initial state set")

        # Check all states
        for state_name in self._states:
            state: State = self._states[state_name]["state"]
            transitions: Dict[str, str] = self._states[state_name]["transitions"]

            outcomes = state.get_outcomes()

            if strict_mode:
                # Check if all outcomes of the state are in transitions
                for o in outcomes:
                    if o not in set(list(transitions.keys()) + list(self.get_outcomes())):
                        raise KeyError(
                            f"State '{state_name}' outcome '{o}' not registered in transitions"
                        )

                    # Outcomes of the state that are in outcomes of the state machine do not need transitions
                    elif o in self.get_outcomes():
                        terminal_outcomes.append(o)

            # If state is a state machine, validate it
            if isinstance(state, StateMachine):
                state.validate(strict_mode)

            # Add terminal outcomes
            terminal_outcomes.extend([transitions[key] for key in transitions])

        # Check terminal outcomes for the state machine
        terminal_outcomes = set(terminal_outcomes)

        if strict_mode:
            # Check if all outcomes of the state machine are in the terminal outcomes
            for o in self.get_outcomes():
                if o not in terminal_outcomes:
                    raise KeyError(f"Target outcome '{o}' not registered in transitions")

        # Check if all terminal outcomes are states or outcomes of the state machine
        for o in terminal_outcomes:
            if o not in set(list(self._states.keys()) + list(self.get_outcomes())):
                raise KeyError(
                    f"State machine outcome '{o}' not registered as outcome neither state"
                )

        # State machine has been validated
        self._validated = True

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the state machine starting from the initial state.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for sharing state.

        Returns:
            str: The final outcome of the state machine execution.

        Raises:
            RuntimeError: If the execution is canceled unexpectedly.
            KeyError: If an outcome does not belong to a state or the state machine.
        """
        self.validate()

        yasmin.YASMIN_LOG_INFO(
            f"Executing state machine with initial state '{self._start_state}'"
        )
        self._call_start_cbs(blackboard, self._start_state)

        self.__set_current_state(self._start_state)

        while not self.is_canceled():
            state = self._states[self.get_current_state()]
            if self.get_current_state() in self.__remappings:
                blackboard.remappings = self.__remappings[self.get_current_state()]
            else:
                blackboard.remappings = dict()
            outcome = state["state"](blackboard)
            old_outcome = outcome
            # Check if outcome belongs to state
            if outcome not in state["state"].get_outcomes():
                raise KeyError(
                    f"Outcome '{outcome}' is not registered in state {self.__current_state}"
                )

            # Translate outcome using transitions
            if outcome in state["transitions"]:
                outcome = state["transitions"][outcome]

            # Outcome is an outcome of the state machine
            if outcome in self.get_outcomes():
                self.__set_current_state("")

                yasmin.YASMIN_LOG_INFO(f"State machine ends with outcome '{outcome}'")
                self._call_end_cbs(blackboard, outcome)
                return outcome

            # Outcome is a state
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

                self.__set_current_state(outcome)

            # Outcome is not in the state machine
            else:
                raise KeyError(
                    f"Outcome '{outcome}' is not a state nor a state machine outcome"
                )

        raise RuntimeError(f"Ending canceled state machine '{self}' with bad transition")

    def cancel_state(self) -> None:
        """
        Cancels the current state and any associated operations.

        Overrides the cancel_state method from the parent State class.
        """

        if self.is_running():
            super().cancel_state()
            current_state = self.get_current_state()

            while not current_state:
                self.__current_state_event.clear()
                self.__current_state_event.wait()
                current_state = self.get_current_state()

            if current_state:
                self._states[current_state]["state"].cancel_state()

    def __str__(self) -> str:
        """
        Returns a string representation of the state machine, listing all states and their types.

        Returns:
            str: A string representation of the state machine.
        """
        result = "State Machine ["
        keys = sorted(list(self._states.keys()))

        for i, key in enumerate(keys):
            result += f"{key} ({self._states[key]['state']})"

            if i < len(keys) - 1:
                result += ", "

        result += "]"
        return result
