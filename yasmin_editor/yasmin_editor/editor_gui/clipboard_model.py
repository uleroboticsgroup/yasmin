# Copyright (C) 2026 Maik Knof
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

from __future__ import annotations

from typing import Union
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.state_machine import StateMachine

ContainerModel = Union[StateMachine, Concurrence, OrthogonalState]


def get_container_kind(model: ContainerModel) -> str:
    """Return the logical container kind used by clipboard workflows."""

    if isinstance(model, OrthogonalState):
        return "orthogonal"
    return "concurrence" if isinstance(model, Concurrence) else "state_machine"


def create_clipboard_container(kind: str, *, name: str = "Shelf") -> ContainerModel:
    """Create an empty clipboard container with the requested semantics."""

    if kind in ("concurrence", "orthogonal"):
        cls = Concurrence if kind == "concurrence" else OrthogonalState
        return cls(name=name)
    return StateMachine(name=name)


def is_container_empty(model: ContainerModel) -> bool:
    """Return whether a clipboard container currently has any staged content."""

    if model.states:
        return False
    if model.outcomes:
        return False
    if model.text_blocks:
        return False
    if isinstance(model, StateMachine):
        return not any(model.transitions.values())
    return not any(model.outcome_map.values())
