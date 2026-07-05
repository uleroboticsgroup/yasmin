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

from typing import Iterable, List, Set
from yasmin_editor.dataclass_compat import dataclass, field

_STATE_ITEM_CLASS_NAMES = frozenset({"StateNode", "ContainerStateNode"})
_FINAL_OUTCOME_ITEM_CLASS_NAMES = frozenset({"FinalOutcomeNode"})
_TEXT_BLOCK_ITEM_CLASS_NAMES = frozenset({"TextBlockNode"})
_CONNECTION_ITEM_CLASS_NAMES = frozenset({"ConnectionLine"})


@dataclass(slots=True)
class SceneSelection:
    """Grouped editor-scene selection data."""

    states: List[object] = field(default_factory=list)
    final_outcomes: List[object] = field(default_factory=list)
    text_blocks: List[object] = field(default_factory=list)
    connections: List[object] = field(default_factory=list)
    state_names: Set[str] = field(default_factory=set)
    outcome_instance_ids: Set[str] = field(default_factory=set)
    text_models: List[object] = field(default_factory=list)

    @property
    def is_empty(self) -> bool:
        return not (
            self.states or self.final_outcomes or self.text_blocks or self.connections
        )


def _class_name_hierarchy(item: object) -> Set[str]:
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
