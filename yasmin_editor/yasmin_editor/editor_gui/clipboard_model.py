# Copyright (C) 2026 Maik Knof
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

from __future__ import annotations

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.state_machine import StateMachine

ContainerModel = StateMachine | Concurrence | OrthogonalState


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
