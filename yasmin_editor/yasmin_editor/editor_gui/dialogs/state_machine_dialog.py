# Copyright (C) 2025 Miguel Ángel González Santamarta
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

"""Dialog for creating and editing state machine containers."""

from typing import Dict, List, Optional, Tuple

from PyQt5.QtWidgets import QComboBox, QDialog, QLabel

from yasmin_editor.editor_gui.dialogs.container_dialog_base import (
    ContainerDialogBase,
)


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
        parent: Optional[QDialog] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
    ) -> None:
        self.start_state_combo: Optional[QComboBox] = None
        self.start_state_label: Optional[QLabel] = None
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
        self.start_state_label = QLabel("Start State:")
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
