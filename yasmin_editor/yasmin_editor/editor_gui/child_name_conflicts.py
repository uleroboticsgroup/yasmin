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

from typing import Iterable, Sequence, Union


def _has_child_name_conflict(
    proposed_name: str,
    *,
    current_name: Union[str, None],
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
    current_name: Union[str, None] = None,
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
    current_name: Union[str, None],
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
