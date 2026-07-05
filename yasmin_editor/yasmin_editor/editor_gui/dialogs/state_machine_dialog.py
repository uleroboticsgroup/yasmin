# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import Dict, List, Optional, Tuple
from yasmin_editor.qt_compat import QtWidgets
from yasmin_editor.editor_gui.dialogs.container_dialog_base import ContainerDialogBase


class StateMachineDialog(ContainerDialogBase):
    """Dialog for creating/editing State Machine containers."""

    def __init__(
        self,
        name: str = "",
        outcomes: Optional[List[str]] = None,
        start_state: Optional[str] = None,
        remappings: Optional[Dict[str, str]] = None,
        child_states: Optional[List[str]] = None,
        edit_mode: bool = False,
        parent: Optional[QtWidgets.QDialog] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
    ) -> None:
        self.start_state_combo: Optional[QtWidgets.QComboBox] = None
        self.start_state_label: Optional[QtWidgets.QLabel] = None
        self._child_states = child_states
        self._start_state = start_state
        self.defaults = defaults or []

        super().__init__(
            window_title="Edit State Machine" if edit_mode else "Add State Machine",
            name=name,
            name_placeholder="Enter state machine name (required)",
            outcomes=outcomes,
            remappings=remappings,
            description=description,
            edit_mode=edit_mode,
            parent=parent,
        )

    def _create_selector_row(self) -> None:
        """Create the state machine start-state selector."""
        self.start_state_label = QtWidgets.QLabel("Start State:")
        self.start_state_combo = self.create_optional_combo(
            values=self._child_states,
            current_value=self._start_state,
            enabled_in_add_mode=False,
        )
        self.layout.addRow(self.start_state_label, self.start_state_combo)

    def get_state_machine_data(
        self,
    ) -> Optional[
        Tuple[str, List[str], Optional[str], Dict[str, str], str, List[Dict[str, str]]]
    ]:
        """Return the normalized dialog content for a state machine."""
        name = self.validate_name("State machine")
        if name is None or self.start_state_combo is None:
            return None

        return (
            name,
            self.parse_outcomes(),
            self.combo_value_or_none(self.start_state_combo),
            self.parse_remappings(),
            self.parse_description(),
            list(self.defaults),
        )
