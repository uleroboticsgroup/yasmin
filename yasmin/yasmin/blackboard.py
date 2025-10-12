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

from typing import Any, Dict
from threading import RLock as Lock

import yasmin


class Blackboard(object):
    """
    A thread-safe storage for key-value pairs of varying types.

    The Blackboard class allows storing, retrieving, and managing
    values associated with string keys in a thread-safe manner using
    a recursive mutex.

    Attributes:
        _data (Dict[str, Any]): Storage for key-value pairs.
        __lock (Lock): Mutex for thread safety.
        __remapping (Dict[str, str]): Storage for key remappings.
    """

    def __init__(self, init: Dict[str, Any] = None) -> None:
        """
        Default constructor for Blackboard.

        Args:
            init (Dict[str, Any], optional): A dictionary to initialize the blackboard with.
                If None, the blackboard starts empty.
        """

        ## Mutex for thread safety.
        self.__lock: Lock = Lock()
        ## Storage for key-value pairs.
        self._data: Dict[str, Any] = {}
        ## Storage for key remappings.
        self.__remapping: Dict[str, str] = {}

        if init is not None:
            self._data.update(init)  # Initialize with provided data

    def __getitem__(self, key: str) -> Any:
        """
        Retrieve a value from the blackboard.

        Args:
            key (str): The key associated with the value.

        Returns:
            Any: The value associated with the specified key.

        Raises:
            KeyError: If the key does not exist.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Getting '{key}' from the blackboard")

        with self.__lock:
            if not self.__contains__(key):
                raise KeyError(f"Element '{key}' does not exist in the blackboard")

            return self._data[self.__remap(key)]

    def __setitem__(self, key: str, value: Any) -> None:
        """
        Set a value in the blackboard.

        Args:
            key (str): The key to associate with the value.
            value (Any): The value to store.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Setting '{key}' in the blackboard")

        with self.__lock:
            self._data[key] = value

    def __delitem__(self, key: str) -> None:
        """
        Remove a value from the blackboard.

        Args:
            key (str): The key associated with the value to remove.

        Raises:
            KeyError: If the key is not found in the blackboard.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Removing '{key}' from the blackboard")

        with self.__lock:
            del self._data[self.__remap(key)]

    def __contains__(self, key: str) -> bool:
        """
        Check if a key exists in the blackboard.

        Args:
            key (str): The key to check.

        Returns:
            bool: True if the key exists, false otherwise.
        """
        yasmin.YASMIN_LOG_DEBUG(f"Checking if '{key}' is in the blackboard")

        with self.__lock:
            return self.__remap(key) in self._data

    def __len__(self) -> int:
        """
        Get the number of key-value pairs in the blackboard.

        Returns:
            int: The size of the blackboard.
        """
        with self.__lock:
            return len(self._data)

    def __repr__(self) -> str:
        """
        Convert the contents of the blackboard to a string.

        Returns:
            str: A string representation of the blackboard.
        """
        with self.__lock:
            return repr(self._data)

    @property
    def remappings(self) -> Dict[str, str]:
        """
        Get the remapping of the blackboard.

        Returns:
            Dict[str, str]: The remapping of the blackboard.
        """
        return self.__remapping

    @remappings.setter
    def remappings(self, remapping: Dict[str, str]) -> None:
        """
        Set the remapping of the blackboard.

        Args:
            remapping (Dict[str, str]): The remapping to set.
        """
        self.__remapping = remapping

    def __remap(self, key: str) -> str:
        """
        Internal method that acquires the mapped key. In the case the key is
        not remapped, returns the arg key.

        Args:
            key (str): The key to be remapped.

        Returns:
            str: The remapped key or if is not remapped, the own key.
        """
        return self.remappings[key] if key in self.remappings.keys() else key
