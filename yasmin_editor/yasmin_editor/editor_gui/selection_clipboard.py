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
"""Compatibility exports for selection clipboard helpers."""

from yasmin_editor.editor_gui.selection_bundle_ops import (
    collect_selection_bundle,
    get_bundle_bounds,
    paste_bundle_into_model,
    remove_selection_from_model,
)
from yasmin_editor.editor_gui.selection_models import (
    ContainerModel,
    OutcomePlacementSnapshot,
    OutcomeRuleSnapshot,
    SelectionBundle,
)
from yasmin_editor.editor_gui.selection_names import increment_name

__all__ = [
    "ContainerModel",
    "OutcomePlacementSnapshot",
    "OutcomeRuleSnapshot",
    "SelectionBundle",
    "collect_selection_bundle",
    "get_bundle_bounds",
    "increment_name",
    "paste_bundle_into_model",
    "remove_selection_from_model",
]
