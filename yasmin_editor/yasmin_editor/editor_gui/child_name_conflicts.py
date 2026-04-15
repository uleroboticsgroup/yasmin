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
"""Pure helpers for child-state and final-outcome name conflict checks."""

from __future__ import annotations

from typing import Iterable, Sequence


def _has_child_name_conflict(
    proposed_name: str,
    *,
    current_name: str | None,
    sibling_state_names: Iterable[str],
    sibling_outcome_names: Sequence[str],
) -> bool:
    """Return whether one child item name collides with siblings in a container."""

    if current_name is not None and proposed_name == current_name:
        return False

    if proposed_name in set(sibling_state_names):
        return True

    return any(
        outcome_name == proposed_name and outcome_name != current_name
        for outcome_name in sibling_outcome_names
    )


def has_state_name_conflict(
    proposed_name: str,
    *,
    sibling_state_names: Iterable[str],
    sibling_outcome_names: Sequence[str],
    current_name: str | None = None,
) -> bool:
    """Return whether a child-state name collides with sibling states or outcomes."""

    return _has_child_name_conflict(
        proposed_name,
        current_name=current_name,
        sibling_state_names=sibling_state_names,
        sibling_outcome_names=sibling_outcome_names,
    )


def has_final_outcome_name_conflict(
    proposed_name: str,
    *,
    current_name: str | None,
    sibling_state_names: Iterable[str],
    sibling_outcome_names: Sequence[str],
) -> bool:
    """Return whether a final-outcome name collides with sibling states or outcomes."""

    return _has_child_name_conflict(
        proposed_name,
        current_name=current_name,
        sibling_state_names=sibling_state_names,
        sibling_outcome_names=sibling_outcome_names,
    )
