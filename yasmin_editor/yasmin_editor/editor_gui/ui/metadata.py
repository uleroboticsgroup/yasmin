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

from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


def _build_metadata_row(label: QLabel, field) -> QWidget:
    """Create one compact metadata row with a label and one expanding field."""

    row_widget = QWidget()
    row_layout = QHBoxLayout(row_widget)
    row_layout.setContentsMargins(0, 0, 0, 0)

    label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
    row_layout.addWidget(label)

    field.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    row_layout.addWidget(field)
    return row_widget


def build_metadata_widget(editor) -> QWidget:
    """Create the root container metadata controls."""

    widget = QWidget()
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(4)

    editor.root_sm_name_edit = QLineEdit()
    editor.root_sm_name_edit.setProperty("flatInput", True)
    editor.root_sm_name_edit.setPlaceholderText("Enter container name...")
    editor.root_sm_name_edit.textChanged.connect(editor.on_root_sm_name_changed)
    editor.root_sm_name_label = QLabel("<b>State Machine Name:</b>")
    layout.addWidget(
        _build_metadata_row(editor.root_sm_name_label, editor.root_sm_name_edit)
    )

    editor.start_state_combo = QComboBox()
    editor.start_state_combo.setProperty("flatInput", True)
    editor.start_state_combo.addItem("(None)")
    editor.start_state_combo.currentTextChanged.connect(editor.on_start_state_changed)
    editor.start_state_label = QLabel("<b>Start State:</b>")
    layout.addWidget(
        _build_metadata_row(editor.start_state_label, editor.start_state_combo)
    )

    editor.root_sm_description_edit = QLineEdit()
    editor.root_sm_description_edit.setProperty("flatInput", True)
    editor.root_sm_description_edit.setPlaceholderText("Enter container description...")
    editor.root_sm_description_edit.textChanged.connect(
        editor.on_root_sm_description_changed
    )
    layout.addWidget(
        _build_metadata_row(
            QLabel("<b>Description:</b>"), editor.root_sm_description_edit
        )
    )
    return widget
