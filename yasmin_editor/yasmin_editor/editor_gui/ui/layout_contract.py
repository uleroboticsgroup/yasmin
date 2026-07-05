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

from yasmin_editor.dataclass_compat import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True, slots=True)
class SplitterLayoutSpec:
    """Layout defaults applied to a two-column QSplitter."""

    initial_sizes: Tuple[int, int]
    stretch_factors: Tuple[int, int]
    children_collapsible: bool = True


MAIN_EDITOR_SPLITTER_SPEC = SplitterLayoutSpec(
    initial_sizes=(260, 1040),
    stretch_factors=(0, 1),
    children_collapsible=True,
)


def _apply_stretch_factors(splitter, stretch_factors: Iterable[int]) -> None:
    """Apply stretch factors to the splitter in index order."""

    for index, factor in enumerate(stretch_factors):
        splitter.setStretchFactor(index, factor)


def apply_splitter_layout(
    splitter, spec: SplitterLayoutSpec = MAIN_EDITOR_SPLITTER_SPEC
) -> None:
    """Apply the canonical editor splitter defaults to a QSplitter-like object."""

    splitter.setChildrenCollapsible(spec.children_collapsible)
    splitter.setSizes(list(spec.initial_sizes))
    _apply_stretch_factors(splitter, spec.stretch_factors)
