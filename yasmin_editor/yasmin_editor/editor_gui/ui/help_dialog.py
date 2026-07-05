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

from yasmin_editor.qt_compat import QtWidgets, exec_dialog
from yasmin_editor.editor_gui.ui.help_content import HELP_HTML

HELP_DIALOG_TITLE = "YASMIN Editor Help"
HELP_DIALOG_MIN_SIZE = (600, 500)
HELP_DIALOG_MAX_SIZE = (800, 600)


def build_help_dialog(parent) -> QtWidgets.QDialog:
    """Build the help dialog without executing it.

    Returning the dialog makes the construction path testable while keeping the
    public ``show_help_dialog`` entrypoint small and UI-focused.
    """

    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle(HELP_DIALOG_TITLE)
    dialog.setMinimumSize(*HELP_DIALOG_MIN_SIZE)
    dialog.setMaximumSize(*HELP_DIALOG_MAX_SIZE)

    layout = QtWidgets.QVBoxLayout(dialog)
    text_browser = QtWidgets.QTextBrowser()
    text_browser.setHtml(HELP_HTML)
    text_browser.setOpenExternalLinks(False)
    layout.addWidget(text_browser)

    button_layout = QtWidgets.QHBoxLayout()
    button_layout.addStretch()
    ok_button = QtWidgets.QPushButton("OK")
    ok_button.clicked.connect(dialog.accept)
    ok_button.setDefault(True)
    button_layout.addWidget(ok_button)
    layout.addLayout(button_layout)
    return dialog


def show_help_dialog(parent) -> None:
    """Open the modal help dialog."""

    exec_dialog(build_help_dialog(parent))
