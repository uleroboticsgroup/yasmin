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

"""Python bindings for yasmin::blackboard::Blackboard"""

from typing import Any, Dict

class Blackboard:
    """
    A blackboard for storing and retrieving key-value pairs.

    The Blackboard class provides a dictionary-like interface for storing
    and retrieving data with support for key remappings.
    """

    def __init__(self) -> None:
        """Initialize a new Blackboard instance."""
        ...

    def set(self, key: str, value: Any) -> None:
        """
        Set a value in the blackboard.

        Args:
            key: The key to set
            value: The value to store
        """
        ...

    def __setitem__(self, key: str, value: Any) -> None:
        """
        Set a value in the blackboard using dictionary syntax.

        Args:
            key: The key to set
            value: The value to store
        """
        ...

    def get(self, key: str) -> Any:
        """
        Get a value from the blackboard.

        Args:
            key: The key to retrieve

        Returns:
            The value associated with the key

        Raises:
            KeyError: If the key does not exist
        """
        ...

    def __getitem__(self, key: str) -> Any:
        """
        Get a value from the blackboard using dictionary syntax.

        Args:
            key: The key to retrieve

        Returns:
            The value associated with the key

        Raises:
            KeyError: If the key does not exist
        """
        ...

    def remove(self, key: str) -> None:
        """
        Remove a value from the blackboard.

        Args:
            key: The key to remove

        Raises:
            KeyError: If the key does not exist
        """
        ...

    def __delitem__(self, key: str) -> None:
        """
        Remove a value from the blackboard using dictionary syntax.

        Args:
            key: The key to remove

        Raises:
            KeyError: If the key does not exist
        """
        ...

    def contains(self, key: str) -> bool:
        """
        Check if a key exists in the blackboard.

        Args:
            key: The key to check

        Returns:
            True if the key exists, False otherwise
        """
        ...

    def __contains__(self, key: str) -> bool:
        """
        Check if a key exists in the blackboard using 'in' operator.

        Args:
            key: The key to check

        Returns:
            True if the key exists, False otherwise
        """
        ...

    def size(self) -> int:
        """
        Get the number of key-value pairs in the blackboard.

        Returns:
            The number of entries
        """
        ...

    def __len__(self) -> int:
        """
        Get the number of key-value pairs in the blackboard.

        Returns:
            The number of entries
        """
        ...

    def to_string(self) -> str:
        """
        Convert the blackboard to a string representation.

        Returns:
            String representation of the blackboard
        """
        ...

    def __str__(self) -> str:
        """
        Convert the blackboard to a string representation.

        Returns:
            String representation of the blackboard
        """
        ...

    def set_remappings(self, remappings: Dict[str, str]) -> None:
        """
        Set the key remappings.

        Args:
            remappings: Dictionary mapping original keys to remapped keys
        """
        ...

    def get_remappings(self) -> Dict[str, str]:
        """
        Get the key remappings.

        Returns:
            Dictionary of key remappings
        """
        ...
