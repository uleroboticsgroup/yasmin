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

from typing import Callable, List, Set, overload
from yasmin.state import State
from yasmin.blackboard import Blackboard

class CbState(State):
    @overload
    def __init__(
        self, outcomes: Set[str], callback: Callable[[Blackboard], str]
    ) -> None: ...
    @overload
    def __init__(
        self, outcomes: List[str], callback: Callable[[Blackboard], str]
    ) -> None: ...
    def execute(self, blackboard: Blackboard) -> str: ...
    def to_string(self) -> str: ...
    def __str__(self) -> str: ...
    def __call__(self, blackboard: Blackboard) -> str: ...
