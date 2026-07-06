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

from collections.abc import Iterable, Mapping
from typing import Protocol, Union

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.state_machine import StateMachine


class _PointLike(Protocol):
    def x(self) -> float: ...

    def y(self) -> float: ...


class _NamedView(Protocol):
    name: str

    def pos(self) -> _PointLike: ...


class _OutcomeView(_NamedView, Protocol):
    instance_id: Union[str, None]


class _TextBlockModel(Protocol):
    x: float
    y: float
    content: str


class _TextBlockView(Protocol):
    model: _TextBlockModel
    content: str

    def pos(self) -> _PointLike: ...


def sync_container_layout_from_views(
    container_model: Union[StateMachine, Concurrence],
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
