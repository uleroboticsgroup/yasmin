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

"""Python bindings for yasmin::StateMachine"""

from typing import Callable, Dict, List, Set, Union, overload, Any
from yasmin.state import State
from yasmin.blackboard import Blackboard

class StateMachine(State):
    """
    A hierarchical state machine that can contain other states.

    StateMachine is a state that contains and manages other states, allowing
    for hierarchical composition of behaviors. It manages state transitions
    and callbacks for state machine lifecycle events.
    """

    @overload
    def __init__(self, outcomes: Set[str]) -> None:
        """
        Initialize a StateMachine with a set of outcomes.

        Args:
            outcomes: Set of possible outcome strings
        """
        ...

    @overload
    def __init__(self, outcomes: List[str]) -> None:
        """
        Initialize a StateMachine with a list of outcomes.

        Args:
            outcomes: List of possible outcome strings
        """
        ...

    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = {},
        remappings: Dict[str, str] = {},
    ) -> None:
        """
        Add a state to the state machine.

        Args:
            name: Name identifier for the state
            state: The state instance to add
            transitions: Dictionary mapping state outcomes to next state names or machine outcomes
            remappings: Dictionary mapping blackboard keys for this state
        """
        ...

    def set_name(self, name: str) -> None:
        """
        Set the name of the state machine.

        Args:
            name: The name for this state machine
        """
        ...

    def get_name(self) -> str:
        """
        Get the name of the state machine.

        Returns:
            The state machine name
        """
        ...

    def set_start_state(self, state_name: str) -> None:
        """
        Set the initial state for the state machine.

        Args:
            state_name: Name of the state to start execution with
        """
        ...

    def get_start_state(self) -> str:
        """
        Get the name of the start state.

        Returns:
            Name of the start state
        """
        ...

    def get_states(self) -> Dict[str, Dict[str, Any]]:
        """
        Get all states with their transitions (Python compatibility format).

        Returns a dictionary in the format:
        {
            "state_name": {
                "state": State instance,
                "transitions": {"outcome": "next_state_or_outcome"}
            }
        }

        Returns:
            Dictionary of states with their configuration
        """
        ...

    def _get_states_cpp(self) -> Dict[str, State]:
        """
        Get all states in the state machine (C++ format).

        Returns:
            Dictionary mapping state names to State instances
        """
        ...

    def get_transitions(self) -> Dict[str, Dict[str, str]]:
        """
        Get all transitions in the state machine.

        Returns:
            Dictionary mapping state names to their transition dictionaries
        """
        ...

    def get_current_state(self) -> str:
        """
        Get the name of the current state being executed.

        Returns:
            Name of the current state, or empty string if not executing
        """
        ...

    def add_start_cb(
        self, cb: Callable[[Blackboard, str, List[str]], None], args: List[str] = []
    ) -> None:
        """
        Add a callback to be called when the state machine starts.

        Args:
            cb: Callback function with signature (blackboard, state_name, args)
            args: List of blackboard keys to pass as arguments to the callback
        """
        ...

    def add_transition_cb(
        self,
        cb: Callable[[Blackboard, str, str, str, List[str]], None],
        args: List[str] = [],
    ) -> None:
        """
        Add a callback to be called during state transitions.

        Args:
            cb: Callback function with signature (blackboard, from_state, to_state, outcome, args)
            args: List of blackboard keys to pass as arguments to the callback
        """
        ...

    def add_end_cb(
        self, cb: Callable[[Blackboard, str, List[str]], None], args: List[str] = []
    ) -> None:
        """
        Add a callback to be called when the state machine ends.

        Args:
            cb: Callback function with signature (blackboard, outcome, args)
            args: List of blackboard keys to pass as arguments to the callback
        """
        ...

    def validate(self, strict_mode: bool = False) -> None:
        """
        Validate the state machine configuration.

        Checks that all states are properly connected and transitions are valid.

        Args:
            strict_mode: If True, perform stricter validation

        Raises:
            RuntimeError: If validation fails
        """
        ...

    def cancel_state(self) -> None:
        """
        Cancel the current state execution.
        """
        ...

    def to_string(self) -> str:
        """
        Convert the state machine to a string representation.

        Returns:
            String representation of the state machine
        """
        ...

    def __str__(self) -> str:
        """
        Convert the state machine to a string representation.

        Returns:
            String representation of the state machine
        """
        ...

    def __call__(self, blackboard: Blackboard) -> str:
        """
        Execute the state machine by calling it like a function.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string indicating the result of execution
        """
        ...
