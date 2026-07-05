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

from yasmin_editor.qt_compat import Qt, QtWidgets

from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import MIN_CLIPBOARD_DOCK_WIDTH
from yasmin_editor.editor_gui.ui.clipboard_panel import build_clipboard_panel


def build_clipboard_dock(editor) -> QtWidgets.QDockWidget:
    """Create the hidden right-side shelf dock."""

    clipboard_panel = build_clipboard_panel(editor)

    clipboard_dock = QtWidgets.QDockWidget("Shelf", editor)
    clipboard_dock.setObjectName("clipboardDock")
    clipboard_dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea)
    clipboard_dock.setFeatures(
        QtWidgets.QDockWidget.DockWidgetFeature.NoDockWidgetFeatures
    )
    clipboard_dock.setTitleBarWidget(QtWidgets.QWidget())
    clipboard_dock.setWidget(clipboard_panel)
    clipboard_dock.setMinimumWidth(MIN_CLIPBOARD_DOCK_WIDTH)
    clipboard_dock.setMaximumWidth(16777215)
    clipboard_dock.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Expanding
    )
    clipboard_dock.hide()

    editor.clipboard_panel = clipboard_panel
    editor.clipboard_dock = clipboard_dock
    editor.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, clipboard_dock)

    return clipboard_dock
