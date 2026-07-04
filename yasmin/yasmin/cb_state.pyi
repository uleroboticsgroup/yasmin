# Copyright (C) 2025 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Callable, List, Set, overload
from yasmin.state import State
from yasmin import Blackboard

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
