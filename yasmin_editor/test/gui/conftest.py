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

# Shared fixtures for the GUI test suite.
#
# The editor fixture stubs optional runtime dependencies so the tests can focus
# on widget interaction and editor behavior inside a headless test run.

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from gui_test_support import FakePluginManager, install_external_dependency_stubs


@pytest.fixture
def editor_window(qapp, monkeypatch, tmp_path):
    # Create an isolated editor instance that does not touch the developer's
    # local configuration and does not require the external runtime stack.
    pytest.importorskip("PyQt5.QtWidgets")

    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))

    # Replace optional dependencies with small test doubles so the tests only
    # exercise the GUI code that is relevant for the editor window.
    install_external_dependency_stubs(monkeypatch)

    editor_module = importlib.import_module("yasmin_editor.editor_gui.yasmin_editor")

    def create_runtime_stub(self) -> None:
        self.runtime = None
        self.runtime_shell = None

    monkeypatch.setattr(
        editor_module.YasminEditor, "_create_runtime", create_runtime_stub
    )
    monkeypatch.setattr(
        editor_module.YasminEditor,
        "_fit_initial_window_to_screen",
        lambda self: None,
    )
    monkeypatch.setattr(
        editor_module.YasminEditor,
        "maybe_save_document_changes",
        lambda self, _label: True,
    )

    # Show one editor instance for the duration of the test case.
    manager = FakePluginManager()
    editor = editor_module.YasminEditor(manager)
    editor.show()
    qapp.processEvents()

    yield editor

    # Reset the dirty flag before shutdown so the close path stays non-modal in
    # headless execution.
    editor.reset_document_dirty_state()
    editor.hide()
    editor.close()
    qapp.processEvents()
