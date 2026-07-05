# Copyright (C) 2026 Miguel Ángel González Santamarta
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
