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

from yasmin_editor.qt_compat import QtWidgets


def _build_metadata_row(label: QtWidgets.QLabel, field) -> QtWidgets.QWidget:
    """Create one compact metadata row with a label and one expanding field."""

    row_widget = QtWidgets.QWidget()
    row_layout = QtWidgets.QHBoxLayout(row_widget)
    row_layout.setContentsMargins(0, 0, 0, 0)

    label.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Preferred
    )
    row_layout.addWidget(label)

    field.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Preferred
    )
    row_layout.addWidget(field)
    return row_widget


def build_metadata_widget(editor) -> QtWidgets.QWidget:
    """Create the root container metadata controls."""

    widget = QtWidgets.QWidget()
    widget.setObjectName("metadataPanel")
    layout = QtWidgets.QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(4)

    editor.root_sm_name_edit = QtWidgets.QLineEdit()
    editor.root_sm_name_edit.setObjectName("rootStateMachineNameEdit")
    editor.root_sm_name_edit.setProperty("flatInput", True)
    editor.root_sm_name_edit.setPlaceholderText("Enter container name...")
    editor.root_sm_name_edit.textChanged.connect(editor.on_root_sm_name_changed)
    editor.root_sm_name_label = QtWidgets.QLabel("<b>State Machine Name:</b>")
    layout.addWidget(
        _build_metadata_row(editor.root_sm_name_label, editor.root_sm_name_edit)
    )

    editor.start_state_combo = QtWidgets.QComboBox()
    editor.start_state_combo.setObjectName("startStateCombo")
    editor.start_state_combo.setProperty("flatInput", True)
    editor.start_state_combo.addItem("(None)")
    editor.start_state_combo.currentTextChanged.connect(editor.on_start_state_changed)
    editor.start_state_label = QtWidgets.QLabel("<b>Start State:</b>")
    layout.addWidget(
        _build_metadata_row(editor.start_state_label, editor.start_state_combo)
    )

    editor.root_sm_description_edit = QtWidgets.QLineEdit()
    editor.root_sm_description_edit.setObjectName("rootStateMachineDescriptionEdit")
    editor.root_sm_description_edit.setProperty("flatInput", True)
    editor.root_sm_description_edit.setPlaceholderText("Enter container description...")
    editor.root_sm_description_edit.textChanged.connect(
        editor.on_root_sm_description_changed
    )
    layout.addWidget(
        _build_metadata_row(
            QtWidgets.QLabel("<b>Description:</b>"), editor.root_sm_description_edit
        )
    )
    return widget
