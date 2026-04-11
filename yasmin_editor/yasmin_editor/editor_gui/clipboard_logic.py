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

"""Qt-free shelf workflow helpers.

The shelf feature combines several editor concerns: container-kind matching,
cross-container link preservation, status text, and split-view labelling. This
module keeps those rules out of the Qt mixin so the behaviour can be covered by
direct tests.
"""

from __future__ import annotations

from typing import Iterable, List, Optional

PASTE_ACTION = "paste"
MOVE_ACTION = "move"
COPY_ACTION = "copy"


def clipboard_kind_label(kind: str) -> str:
    """Return the user-facing shelf container label."""

    return "Concurrence" if kind == "concurrence" else "State Machine"


def replacement_clipboard_message(required_kind: str) -> str:
    """Return the warning shown before replacing shelf contents."""

    label = "concurrence" if required_kind == "concurrence" else "state machine"
    return (
        "The shelf currently contains items from a different container type. "
        f"To preserve the correct transition semantics it needs to switch to a {label} shelf. "
        "Replace the shelf contents?"
    )


def cross_container_link_losses(
    source_kind: str,
    target_kind: str,
    *,
    has_transitions: bool,
    has_outcome_rules: bool,
) -> List[str]:
    """Return the link categories that cannot survive one paste.

    Moving data between a state machine and a concurrence is allowed, but only
    data that both container kinds can represent should survive unchanged.
    """

    if source_kind == target_kind:
        return []

    losses: List[str] = []
    if source_kind == "state_machine" and has_transitions:
        losses.append("state-machine transitions")
    if source_kind == "concurrence" and has_outcome_rules:
        losses.append("concurrence outcome rules")
    return losses


def cross_container_paste_warning(
    source_kind: str,
    target_kind: str,
    *,
    has_transitions: bool,
    has_outcome_rules: bool,
) -> Optional[str]:
    """Return the cross-container warning text or ``None``.

    The editor only needs to warn when the paste changes container kind and the
    selected bundle carries links that the target container cannot encode.
    """

    losses = cross_container_link_losses(
        source_kind,
        target_kind,
        has_transitions=has_transitions,
        has_outcome_rules=has_outcome_rules,
    )
    if not losses:
        return None

    detail_text = " and ".join(losses)
    return (
        "This paste targets a different container type. "
        f"The following container-specific links cannot be preserved: {detail_text}. "
        "Continue anyway?"
    )


def clipboard_operation_status(action: str, *, performed: bool) -> Optional[str]:
    """Return the status-bar text for one shelf workflow.

    ``None`` means the operation did not complete and should stay silent.
    """

    if not performed:
        return None

    labels = {
        COPY_ACTION: "Prepared copied selection",
        PASTE_ACTION: "Placed stored selection",
        MOVE_ACTION: "Moved stored selection",
    }
    return labels.get(action)
