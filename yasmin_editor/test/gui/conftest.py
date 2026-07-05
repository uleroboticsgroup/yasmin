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
    pytest.importorskip("yasmin_editor.qt_compat")

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
