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

"""Python bindings for yasmin::Concurrence"""

from typing import Dict, List
from yasmin.state import State
from yasmin.blackboard import Blackboard

# Type alias for the outcome map
# Maps outcome names to dictionaries mapping state names to required outcomes
OutcomeMap = Dict[str, Dict[str, str]]

class Concurrence(State):
    """
    A state that executes multiple child states concurrently.

    Concurrence executes all child states in parallel and determines the
    overall outcome based on the outcome map configuration. This allows for
    concurrent execution of multiple behaviors.
    """

    def __init__(
        self, states: Dict[str, State], default_outcome: str, outcome_map: OutcomeMap = {}
    ) -> None:
        """
        Initialize a Concurrence state.

        Args:
            states: Dictionary mapping state names to State instances
            default_outcome: The default outcome if no outcome map conditions are met
            outcome_map: Maps concurrence outcomes to conditions on child state outcomes.
                        Format: {"outcome": {"state_name": "required_outcome", ...}, ...}
        """
        ...

    def get_states(self) -> Dict[str, State]:
        """
        Get all states in the concurrence.

        Returns:
            Dictionary mapping state names to State instances
        """
        ...

    def get_outcome_map(self) -> OutcomeMap:
        """
        Get the outcome map for this concurrence state.

        Returns:
            The outcome map configuration
        """
        ...

    def get_default_outcome(self) -> str:
        """
        Get the default outcome for this concurrence state.

        Returns:
            The default outcome string
        """
        ...

    def cancel_state(self) -> None:
        """
        Cancel the current state execution.

        This will cancel all child states that are currently executing.
        """
        ...

    def to_string(self) -> str:
        """
        Convert the concurrence to a string representation.

        Returns:
            String representation of the concurrence
        """
        ...

    def __str__(self) -> str:
        """
        Convert the concurrence to a string representation.

        Returns:
            String representation of the concurrence
        """
        ...

    def __call__(self, blackboard: Blackboard) -> str:
        """
        Execute the concurrence by calling it like a function.

        Args:
            blackboard: The blackboard for reading/writing data

        Returns:
            The outcome string indicating the result of execution
        """
        ...
