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
"""Help dialog construction for the editor window."""

from __future__ import annotations

from PyQt5.QtWidgets import QDialog, QHBoxLayout, QPushButton, QTextBrowser, QVBoxLayout

from yasmin_editor.editor_gui.ui.help_content import HELP_HTML

HELP_DIALOG_TITLE = "YASMIN Editor Help"
HELP_DIALOG_MIN_SIZE = (600, 500)
HELP_DIALOG_MAX_SIZE = (800, 600)


def build_help_dialog(parent) -> QDialog:
    """Build the help dialog without executing it.

    Returning the dialog makes the construction path testable while keeping the
    public ``show_help_dialog`` entrypoint small and UI-focused.
    """

    dialog = QDialog(parent)
    dialog.setWindowTitle(HELP_DIALOG_TITLE)
    dialog.setMinimumSize(*HELP_DIALOG_MIN_SIZE)
    dialog.setMaximumSize(*HELP_DIALOG_MAX_SIZE)

    layout = QVBoxLayout(dialog)
    text_browser = QTextBrowser()
    text_browser.setHtml(HELP_HTML)
    text_browser.setOpenExternalLinks(False)
    layout.addWidget(text_browser)

    button_layout = QHBoxLayout()
    button_layout.addStretch()
    ok_button = QPushButton("OK")
    ok_button.clicked.connect(dialog.accept)
    ok_button.setDefault(True)
    button_layout.addWidget(ok_button)
    layout.addLayout(button_layout)
    return dialog


def show_help_dialog(parent) -> None:
    """Open the modal help dialog."""

    build_help_dialog(parent).exec_()
