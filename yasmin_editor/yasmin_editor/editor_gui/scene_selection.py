#!/usr/bin/env python3
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
"""Helpers for grouping scene selections without depending on PyQt imports."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

_STATE_ITEM_CLASS_NAMES = frozenset({"StateNode", "ContainerStateNode"})
_FINAL_OUTCOME_ITEM_CLASS_NAMES = frozenset({"FinalOutcomeNode"})
_TEXT_BLOCK_ITEM_CLASS_NAMES = frozenset({"TextBlockNode"})
_CONNECTION_ITEM_CLASS_NAMES = frozenset({"ConnectionLine"})


@dataclass(slots=True)
class SceneSelection:
    """Grouped editor-scene selection data."""

    states: list[object] = field(default_factory=list)
    final_outcomes: list[object] = field(default_factory=list)
    text_blocks: list[object] = field(default_factory=list)
    connections: list[object] = field(default_factory=list)
    state_names: set[str] = field(default_factory=set)
    outcome_instance_ids: set[str] = field(default_factory=set)
    text_models: list[object] = field(default_factory=list)

    @property
    def is_empty(self) -> bool:
        return not (
            self.states or self.final_outcomes or self.text_blocks or self.connections
        )


def _class_name_hierarchy(item: object) -> set[str]:
    return {cls.__name__ for cls in type(item).mro()}


def _matches_class_name(item: object, expected_names: frozenset[str]) -> bool:
    return not expected_names.isdisjoint(_class_name_hierarchy(item))


def collect_scene_selection(items: Iterable[object]) -> SceneSelection:
    """Group selected scene items by editor role.

    The helper intentionally avoids importing Qt-dependent graphics item classes,
    which keeps it usable in source-level tests and pure selection helpers.
    """

    selection = SceneSelection()
    for item in items:
        if _matches_class_name(item, _CONNECTION_ITEM_CLASS_NAMES):
            selection.connections.append(item)
            continue

        if _matches_class_name(item, _STATE_ITEM_CLASS_NAMES):
            selection.states.append(item)
            name = getattr(item, "name", "")
            if name:
                selection.state_names.add(name)
            continue

        if _matches_class_name(item, _FINAL_OUTCOME_ITEM_CLASS_NAMES):
            selection.final_outcomes.append(item)
            instance_id = getattr(item, "instance_id", "")
            if instance_id:
                selection.outcome_instance_ids.add(instance_id)
            continue

        if _matches_class_name(item, _TEXT_BLOCK_ITEM_CLASS_NAMES):
            selection.text_blocks.append(item)
            model = getattr(item, "model", None)
            if model is not None:
                selection.text_models.append(model)

    return selection
