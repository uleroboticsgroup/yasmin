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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget

from yasmin_editor.editor_gui.ui.runtime_specs import (
    RUNTIME_BUTTON_SPECS,
    RUNTIME_STATUS_LABEL_MIN_WIDTH,
    RUNTIME_STATUS_LABEL_TEXT,
    RuntimeButtonSpec,
)


def _build_runtime_status_label(editor) -> QLabel:
    """Create the runtime status label shown at the beginning of the bar."""

    editor.runtime_status_label = QLabel(RUNTIME_STATUS_LABEL_TEXT)
    editor.runtime_status_label.setAlignment(Qt.AlignCenter)
    editor.runtime_status_label.setMinimumWidth(RUNTIME_STATUS_LABEL_MIN_WIDTH)
    editor.runtime_status_label.setTextFormat(Qt.PlainText)
    return editor.runtime_status_label


def _build_runtime_button(editor, spec: RuntimeButtonSpec) -> QPushButton:
    """Create one runtime control button from its declarative specification."""

    button = QPushButton(spec.text)
    button.setToolTip(spec.tooltip)
    button.setCheckable(spec.checkable)
    if spec.checkable:
        button.setChecked(spec.checked)
    button.clicked.connect(getattr(editor, spec.callback_name))
    setattr(editor, spec.attribute_name, button)
    return button


def build_runtime_controls_widget(editor) -> QWidget:
    """Create the runtime control bar.

    The runtime controls intentionally keep their original order so the runtime
    workflow stays familiar while the surrounding UI code becomes smaller and
    easier to maintain.
    """

    widget = QWidget()
    editor.runtime_controls_layout = QHBoxLayout(widget)
    editor.runtime_controls_layout.setContentsMargins(0, 0, 0, 0)

    editor.runtime_controls_layout.addWidget(_build_runtime_status_label(editor))

    for spec in RUNTIME_BUTTON_SPECS:
        editor.runtime_controls_layout.addWidget(_build_runtime_button(editor, spec))

    editor.runtime_controls_layout.addStretch()
    return widget
