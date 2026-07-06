# Copyright (C) 2025 Miguel Ángel González Santamarta
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

import random
from typing import Dict, List, Optional, Set, Tuple, Union

from yasmin_editor.qt_compat import QtCore, QtGui, QtWidgets
from yasmin_plugins_manager.plugin_manager import PluginManager

from yasmin_editor.editor_gui.clipboard_model import create_clipboard_container
from yasmin_editor.editor_gui.colors import PALETTE, build_qt_palette, build_stylesheet
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.editor_mixin.editor_blackboard_mixin import (
    EditorBlackboardMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_canvas_mixin import EditorCanvasMixin
from yasmin_editor.editor_gui.editor_mixin.editor_document_mixin import (
    EditorDocumentMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_history_mixin import (
    EditorHistoryMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_clipboard_mixin import (
    EditorClipboardMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_model_mixin import EditorModelMixin
from yasmin_editor.editor_gui.editor_mixin.editor_runtime_mixin import (
    EditorRuntimeMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin import EditorUiMixin
from yasmin_editor.editor_gui.model_adapter import EditorModelAdapter
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.editor_gui.ui.clipboard_dock_sizing import (
    normalize_persisted_clipboard_dock_width,
)
from yasmin_editor.editor_gui.window_sizing import (
    WindowRect,
    build_initial_window_rect,
    choose_preferred_screen_rect,
)
from yasmin_editor.model.state_machine import StateMachine


class YasminEditor(
    EditorRuntimeMixin,
    EditorBlackboardMixin,
    EditorCanvasMixin,
    EditorDocumentMixin,
    EditorClipboardMixin,
    EditorHistoryMixin,
    EditorModelMixin,
    EditorUiMixin,
    QtWidgets.QMainWindow,
):
    """Main editor window for YASMIN state machines.

    Provides a graphical interface for creating, editing, and managing
    hierarchical state machines with support for Python, C++, and XML states.
    """

    def __init__(self, manager: PluginManager) -> None:
        """Initialize the YASMIN editor window.

        Args:
            manager: Plugin manager used to discover available states.
        """
        super().__init__()
        self._apply_theme()

        self.plugin_manager = manager
        self.root_model = StateMachine(name="")
        self.state_nodes: Dict[str, Union[StateNode, ContainerStateNode]] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self.text_blocks: List[TextBlockNode] = []
        self.clipboard_model = create_clipboard_container("state_machine")
        self.clipboard_state_nodes: Dict[str, Union[StateNode, ContainerStateNode]] = {}
        self.clipboard_final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.clipboard_connections: List[ConnectionLine] = []
        self.clipboard_text_blocks: List[TextBlockNode] = []
        self.pending_selection_bundle = None
        self.pending_selection_status_text = ""
        self._blackboard_keys: List[Dict[str, str]] = []
        self._blackboard_key_metadata: Dict[str, Dict[str, str]] = {}
        self._highlight_blackboard_usage = True
        self._show_hidden_blackboard_keys = False

        self.layout_seed = 42
        self.layout_rng = random.Random(self.layout_seed)
        self.current_container_path = [self.root_model]
        self.current_runtime_container_path: List[str] = []
        self._preferred_startup_screen = None
        self._startup_window_shown = False
        self._clipboard_panel_width = self._load_clipboard_panel_width()
        self.current_file_path: Optional[str] = None
        self.runtime_snapshot_file_path: Optional[str] = None
        self.runtime_mode_enabled = False
        self.runtime_auto_follow_enabled = False
        self.runtime_active_path: Tuple[str, ...] = tuple()
        self.runtime_last_transition: Optional[
            Tuple[Tuple[str, ...], Tuple[str, ...], str]
        ] = None
        self.runtime_breakpoints_before: Set[Tuple[str, ...]] = set()

        self.runtime = None
        self.runtime_shell = None
        self._create_runtime()
        self._create_history()
        self._create_document_state()

        self.model_adapter = EditorModelAdapter(self)
        self.create_ui()
        self._fit_initial_window_to_screen()
        self.refresh_clipboard_canvas()
        self.reset_history()
        self.update_document_window_title()

        self.statusBar().showMessage("Loading plugins...")
        QtWidgets.QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    @staticmethod
    def _editor_settings() -> QtCore.QSettings:
        """Return the persistent settings store used by the editor UI."""

        return QtCore.QSettings("yasmin_editor", "yasmin_editor")

    def _load_clipboard_panel_width(self) -> int:
        """Load the persisted shelf width or fall back to a safe default."""

        value = self._editor_settings().value("shelf/width")
        return normalize_persisted_clipboard_dock_width(value)

    def save_clipboard_panel_width(self, width: int) -> None:
        """Persist the current shelf width for future sessions."""

        self._editor_settings().setValue("shelf/width", int(width))

    def _apply_theme(self) -> None:
        """Apply the configured application palette and stylesheet."""
        app = QtWidgets.QApplication.instance()
        if app is None:
            return
        app.setPalette(build_qt_palette(PALETTE))
        app.setStyleSheet(build_stylesheet(PALETTE))

    def _fit_initial_window_to_screen(self) -> None:
        """Clamp the initial editor window geometry to the active screen."""
        app = QtWidgets.QApplication.instance()
        if app is None:
            return

        screens = list(app.screens())
        if not screens:
            return

        current_screen = self.screen()
        if current_screen in screens:
            fallback_index = screens.index(current_screen)
        else:
            primary_screen = app.primaryScreen()
            fallback_index = (
                screens.index(primary_screen) if primary_screen in screens else 0
            )

        screen_rects = [
            WindowRect(
                x=screen.availableGeometry().x(),
                y=screen.availableGeometry().y(),
                width=screen.availableGeometry().width(),
                height=screen.availableGeometry().height(),
            )
            for screen in screens
        ]
        cursor_pos = QtGui.QCursor.pos()
        available = choose_preferred_screen_rect(
            screen_rects,
            cursor_x=cursor_pos.x(),
            cursor_y=cursor_pos.y(),
            fallback_index=fallback_index,
        )
        if available is None:
            return

        selected_index = screen_rects.index(available)
        self._preferred_startup_screen = screens[selected_index]

        rect = build_initial_window_rect(
            available.x,
            available.y,
            available.width,
            available.height,
        )
        self.setGeometry(rect.x, rect.y, rect.width, rect.height)
        window_handle = self.windowHandle()
        if window_handle is not None and self._preferred_startup_screen is not None:
            window_handle.setScreen(self._preferred_startup_screen)

    def show_startup_window(self) -> None:
        """Show the editor on the preferred screen and maximize it immediately."""

        if self._startup_window_shown:
            self.showMaximized()
            return

        self._startup_window_shown = True
        self.winId()
        window_handle = self.windowHandle()
        if window_handle is not None and self._preferred_startup_screen is not None:
            window_handle.setScreen(self._preferred_startup_screen)
            self.setGeometry(self._preferred_startup_screen.availableGeometry())

        self.show()
        QtWidgets.QApplication.processEvents()

        if window_handle is not None and self._preferred_startup_screen is not None:
            window_handle.setScreen(self._preferred_startup_screen)
            self.setGeometry(self._preferred_startup_screen.availableGeometry())

        self.showMaximized()
        self.raise_()
        self.activateWindow()

    @property
    def root_sm_name(self) -> str:
        """Return the name of the root state machine model."""
        return self.root_model.name

    @root_sm_name.setter
    def root_sm_name(self, value: str) -> None:
        """Update the name of the root state machine model."""
        self.root_model.name = value

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle window close event to ensure proper cleanup."""
        if not self.maybe_save_document_changes("closing"):
            event.ignore()
            return

        self._remember_clipboard_panel_width()
        self._shutdown_runtime_shell()
        self._destroy_runtime()
        self._delete_runtime_snapshot()

        self._reset_pending_selection_state()
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()
        self.text_blocks.clear()
        self.clipboard_state_nodes.clear()
        self.clipboard_final_outcomes.clear()
        self.clipboard_connections.clear()
        self.clipboard_text_blocks.clear()

        super().closeEvent(event)
