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
"""Stable splitter defaults for the original editor layout.

The editor only has one main left/right splitter that strongly influences how
"normal" the window feels. This module keeps those defaults declarative so UI
refactors do not quietly drift away from the original proportions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True, slots=True)
class SplitterLayoutSpec:
    """Layout defaults applied to a two-column QSplitter."""

    initial_sizes: tuple[int, int]
    stretch_factors: tuple[int, int]
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
