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

import random
from typing import Dict, List, Optional

from PyQt5.QtCore import QSettings
from PyQt5.QtGui import QCloseEvent, QCursor
from PyQt5.QtWidgets import QApplication, QMainWindow
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
from yasmin_editor.editor_gui.editor_mixin.editor_history_mixin import EditorHistoryMixin
from yasmin_editor.editor_gui.editor_mixin.editor_clipboard_mixin import (
    EditorClipboardMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_model_mixin import EditorModelMixin
from yasmin_editor.editor_gui.editor_mixin.editor_runtime_mixin import EditorRuntimeMixin
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
    QMainWindow,
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
        self.state_nodes: Dict[str, StateNode | ContainerStateNode] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self.text_blocks: List[TextBlockNode] = []
        self.clipboard_model = create_clipboard_container("state_machine")
        self.clipboard_state_nodes: Dict[str, StateNode | ContainerStateNode] = {}
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
        self.runtime_active_path: tuple[str, ...] = tuple()
        self.runtime_last_transition: Optional[
            tuple[tuple[str, ...], tuple[str, ...], str]
        ] = None
        self.runtime_breakpoints_before: set[tuple[str, ...]] = set()

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
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    @staticmethod
    def _editor_settings() -> QSettings:
        """Return the persistent settings store used by the editor UI."""

        return QSettings("yasmin_editor", "yasmin_editor")

    def _load_clipboard_panel_width(self) -> int:
        """Load the persisted shelf width or fall back to a safe default."""

        value = self._editor_settings().value("shelf/width")
        return normalize_persisted_clipboard_dock_width(value)

    def save_clipboard_panel_width(self, width: int) -> None:
        """Persist the current shelf width for future sessions."""

        self._editor_settings().setValue("shelf/width", int(width))

    def _apply_theme(self) -> None:
        """Apply the configured application palette and stylesheet."""
        app = QApplication.instance()
        if app is None:
            return
        app.setPalette(build_qt_palette(PALETTE))
        app.setStyleSheet(build_stylesheet(PALETTE))

    def _fit_initial_window_to_screen(self) -> None:
        """Clamp the initial editor window geometry to the active screen."""
        app = QApplication.instance()
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
        cursor_pos = QCursor.pos()
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
        QApplication.processEvents()

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

    def closeEvent(self, event: QCloseEvent) -> None:
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
