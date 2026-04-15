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
"""Helpers for keeping container layout models in sync with canvas items."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Protocol

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine


class _PointLike(Protocol):
    def x(self) -> float: ...

    def y(self) -> float: ...


class _NamedView(Protocol):
    name: str

    def pos(self) -> _PointLike: ...


class _OutcomeView(_NamedView, Protocol):
    instance_id: str | None


class _TextBlockModel(Protocol):
    x: float
    y: float
    content: str


class _TextBlockView(Protocol):
    model: _TextBlockModel
    content: str

    def pos(self) -> _PointLike: ...


def sync_container_layout_from_views(
    container_model: StateMachine | Concurrence,
    state_views: Mapping[str, _NamedView],
    outcome_views: Iterable[_OutcomeView],
    text_block_views: Iterable[_TextBlockView],
) -> None:
    """Persist the current view positions back into a container layout model."""

    for state_name, state_view in state_views.items():
        position = state_view.pos()
        container_model.layout.set_state_position(
            state_name,
            float(position.x()),
            float(position.y()),
        )

    for outcome_view in outcome_views:
        position = outcome_view.pos()
        container_model.layout.set_outcome_position(
            outcome_view.name,
            float(position.x()),
            float(position.y()),
            instance_id=outcome_view.instance_id or None,
        )

    for text_block_view in text_block_views:
        position = text_block_view.pos()
        text_block_view.model.x = float(position.x())
        text_block_view.model.y = float(position.y())
        text_block_view.model.content = text_block_view.content
