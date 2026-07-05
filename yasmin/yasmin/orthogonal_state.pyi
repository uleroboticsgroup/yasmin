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

from typing import Dict, List, Optional
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin import Blackboard

class RegionDescriptor:
    name: str
    sm: StateMachine

class OrthogonalState(State):
    def __init__(
        self,
        default_outcome: str,
        outcome_map: Dict[str, Dict[str, str]] = ...,
    ) -> None: ...
    def add_region(self, name: str, sm: StateMachine) -> None: ...
    def get_regions(self) -> List[RegionDescriptor]: ...
    def get_outcome_map(self) -> Dict[str, Dict[str, str]]: ...
    def get_default_outcome(self) -> str: ...
    def configure(self) -> None: ...
    def validate(self, strict_mode: bool = False) -> None: ...
    def cancel_state(self) -> None: ...
    def to_string(self) -> str: ...
    def __str__(self) -> str: ...
    def __call__(self, blackboard: Optional[Blackboard] = None) -> str: ...
