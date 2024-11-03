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

from typing import Any, Dict
from threading import Lock

import yasmin


class Blackboard(object):
    """
    A thread-safe blackboard for storing key-value pairs.

    The Blackboard class provides a mechanism to store data in a dictionary-like format,
    allowing for concurrent access and modification through thread-safe operations.

    Attributes:
        _data (Dict[str, Any]): A dictionary holding the data stored in the blackboard.
        __lock (Lock): A threading lock to ensure thread-safe access to the _data attribute.

    Methods:
        __getitem__(key): Retrieve a value associated with the given key.
        __setitem__(key, value): Set a value for the given key.
        __delitem__(key): Remove the value associated with the given key.
        __contains__(key): Check if a key exists in the blackboard.
        __len__(): Return the number of items in the blackboard.
        __repr__(): Return a string representation of the blackboard's data.
    """

    def __init__(self, init: Dict[str, Any] = None) -> None:
        """
        Initializes the Blackboard with an optional initial dictionary.

        Args:
            init (Dict[str, Any], optional): A dictionary to initialize the blackboard with.
                If None, the blackboard starts empty.

        Raises:
            None
        """

        ## A threading lock to ensure thread-safe access to the _data attribute.
        self.__lock: Lock = Lock()
        ## A dictionary holding the data stored in the blackboard.
        self._data: Dict[str, Any] = {}

        if init is not None:
            self._data.update(init)  # Initialize with provided data

    def __getitem__(self, key: str) -> Any:
        """
        Retrieves a value from the blackboard associated with the specified key.

        Args:
            key (str): The key whose value needs to be retrieved.

        Returns:
            Any: The value associated with the key.

        Raises:
            KeyError: If the key is not found in the blackboard.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Getting '{key}' from the blackboard")

        with self.__lock:
            return self._data[key]

    def __setitem__(self, key: str, value: Any) -> None:
        """
        Sets a value in the blackboard for the specified key.

        Args:
            key (str): The key to associate with the value.
            value (Any): The value to be stored in the blackboard.

        Returns:
            None

        Raises:
            None
        """
        yasmin.YASMIN_LOG_DEBUG(f"Setting '{key}' in the blackboard")

        with self.__lock:
            self._data[key] = value

    def __delitem__(self, key: str) -> None:
        """
        Removes the value associated with the specified key from the blackboard.

        Args:
            key (str): The key to be removed.

        Returns:
            None

        Raises:
            KeyError: If the key is not found in the blackboard.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Removing '{key}' from the blackboard")

        with self.__lock:
            del self._data[key]

    def __contains__(self, key: str) -> bool:
        """
        Checks if a specified key exists in the blackboard.

        Args:
            key (str): The key to check for existence.

        Returns:
            bool: True if the key exists, False otherwise.

        Raises:
            None
        """
        yasmin.YASMIN_LOG_DEBUG(f"Checking if '{key}' is in the blackboard")

        with self.__lock:
            return key in self._data

    def __len__(self) -> int:
        """
        Returns the number of items stored in the blackboard.

        Returns:
            int: The count of items in the blackboard.

        Raises:
            None
        """
        with self.__lock:
            return len(self._data)

    def __repr__(self) -> str:
        """
        Returns a string representation of the blackboard's data.

        Returns:
            str: A string representation of the current state of the blackboard.

        Raises:
            None
        """
        with self.__lock:
            return repr(self._data)
