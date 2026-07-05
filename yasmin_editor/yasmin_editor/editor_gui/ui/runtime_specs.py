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

from typing import Tuple
from yasmin_editor.dataclass_compat import dataclass


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

RUNTIME_BUTTON_SPECS: Tuple[RuntimeButtonSpec, ...] = (
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
