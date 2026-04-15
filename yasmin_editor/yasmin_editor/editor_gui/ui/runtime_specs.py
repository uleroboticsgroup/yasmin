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
"""Declarative runtime control definitions.

Keeping the runtime control bar data-driven makes it easier to preserve the
original runtime surface while still refactoring the UI code around it.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RuntimeButtonSpec:
    """Description of one runtime control button."""

    attribute_name: str
    text: str
    callback_name: str
    tooltip: str
    checkable: bool = False
    checked: bool = False


RUNTIME_STATUS_LABEL_MIN_WIDTH = 120
RUNTIME_STATUS_LABEL_TEXT = "Ready"
RUNTIME_AUTO_FOLLOW_ON_TEXT = "Auto Follow: ON"

RUNTIME_BUTTON_SPECS: tuple[RuntimeButtonSpec, ...] = (
    RuntimeButtonSpec(
        attribute_name="runtime_play_button",
        text="Play",
        callback_name="on_runtime_play_clicked",
        tooltip="Start the runtime or resume execution after a pause.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_pause_button",
        text="Request Pause",
        callback_name="on_runtime_pause_clicked",
        tooltip="Pause at next transition.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_step_button",
        text="Play Once",
        callback_name="on_runtime_step_clicked",
        tooltip="Execute exactly one state and pause before the following state starts.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_cancel_state_button",
        text="Cancel State",
        callback_name="on_runtime_cancel_state_clicked",
        tooltip="Request cancellation of the currently active state.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_cancel_sm_button",
        text="Cancel State Machine",
        callback_name="on_runtime_cancel_sm_clicked",
        tooltip="Request cancellation of the complete runtime state machine.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_restart_button",
        text="Restart",
        callback_name="restart_runtime_mode",
        tooltip="Recreate the runtime state machine from a fresh XML snapshot.",
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_auto_follow_button",
        text=RUNTIME_AUTO_FOLLOW_ON_TEXT,
        callback_name="on_runtime_auto_follow_toggled",
        tooltip="Automatically navigate into nested state machines so the active state remains visible.",
        checkable=True,
        checked=True,
    ),
    RuntimeButtonSpec(
        attribute_name="runtime_shell_button",
        text="Interactive Shell",
        callback_name="on_runtime_shell_clicked",
        tooltip=(
            "Open an interactive shell with access to bb, sm, current_state, "
            "and last_state. The shell stays available during execution. "
            "Recommendation: avoid modifying the blackboard while the runtime "
            "is running."
        ),
    ),
)
