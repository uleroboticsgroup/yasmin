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

"""Dialog for creating and editing concurrence containers."""

from typing import Dict, List, Optional, Tuple

from PyQt5.QtWidgets import QComboBox, QDialog, QLabel

from yasmin_editor.editor_gui.dialogs.container_dialog_base import ContainerDialogBase


class ConcurrenceDialog(ContainerDialogBase):
    """Dialog for creating/editing Concurrence containers."""

    def __init__(
        self,
        name: str = "",
        outcomes: Optional[List[str]] = None,
        default_outcome: Optional[str] = None,
        remappings: Optional[Dict[str, str]] = None,
        final_outcomes: Optional[List[str]] = None,
        edit_mode: bool = False,
        parent: Optional[QDialog] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
    ) -> None:
        self.default_outcome_combo: Optional[QComboBox] = None
        self.default_outcome_label: Optional[QLabel] = None
        self._final_outcomes = final_outcomes
        self._default_outcome = default_outcome
        self.defaults = defaults or []

        super().__init__(
            window_title="Edit Concurrence" if edit_mode else "Add Concurrence",
            name=name,
            name_placeholder="Enter concurrence name (required)",
            outcomes=outcomes,
            remappings=remappings,
            description=description,
            edit_mode=edit_mode,
            parent=parent,
        )

    def _create_selector_row(self) -> None:
        """Create the default-outcome selector for the concurrence."""
        self.default_outcome_label = QLabel("Default Outcome:")
        self.default_outcome_combo = self.create_optional_combo(
            values=self._final_outcomes,
            current_value=self._default_outcome,
            enabled_in_add_mode=False,
        )
        self.layout.addRow(self.default_outcome_label, self.default_outcome_combo)

    def get_concurrence_data(
        self,
    ) -> Optional[
        Tuple[str, List[str], Optional[str], Dict[str, str], str, List[Dict[str, str]]]
    ]:
        """Return the normalized dialog content for a concurrence."""
        name = self.validate_name("Concurrence")
        if name is None or self.default_outcome_combo is None:
            return None

        return (
            name,
            self.parse_outcomes(),
            self.combo_value_or_none(self.default_outcome_combo),
            self.parse_remappings(),
            self.parse_description(),
            list(self.defaults),
        )
