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


from typing import Any, Dict
from threading import Lock


class Blackboard(object):

    def __init__(self, init: Dict[str, Any] = None) -> None:
        self.__lock = Lock()
        self._data = {}
        if init is not None:
            self._data.update(init)

    def __getitem__(self, key) -> Any:
        with self.__lock:
            return self._data[key]

    def __setitem__(self, key, value) -> None:
        with self.__lock:
            self._data[key] = value

    def __delitem__(self, key) -> None:
        with self.__lock:
            del self._data[key]

    def __contains__(self, key) -> bool:
        with self.__lock:
            return key in self._data

    def __len__(self) -> int:
        with self.__lock:
            return len(self._data)

    def __repr__(self) -> str:
        with self.__lock:
            return repr(self._data)
