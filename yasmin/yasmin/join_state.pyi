# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from typing import Optional
from yasmin.state import State
from yasmin import Blackboard

class JoinState(State):
    def __init__(self) -> None: ...
    def __init__(self, sync_id: str, outcome: str = "joined") -> None: ...
    def get_sync_id(self) -> str: ...
    def to_string(self) -> str: ...
    def __str__(self) -> str: ...
    def __call__(self, blackboard: Optional[Blackboard] = None) -> str: ...
