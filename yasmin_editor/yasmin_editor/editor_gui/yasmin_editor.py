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

import os
import random
from typing import Dict, List, Optional

from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import QApplication, QMainWindow
from yasmin_plugins_manager.plugin_manager import PluginManager

from yasmin_editor.editor_gui.colors import PALETTE, build_qt_palette, build_stylesheet
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.editor_mixin.editor_blackboard_mixin import (
    EditorBlackboardMixin,
)
from yasmin_editor.editor_gui.editor_mixin.editor_canvas_mixin import EditorCanvasMixin
from yasmin_editor.editor_gui.editor_mixin.editor_model_mixin import EditorModelMixin
from yasmin_editor.editor_gui.editor_mixin.editor_runtime_mixin import EditorRuntimeMixin
from yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin import EditorUiMixin
from yasmin_editor.editor_gui.model_adapter import EditorModelAdapter
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.model.state_machine import StateMachine


class YasminEditor(
    EditorRuntimeMixin,
    EditorBlackboardMixin,
    EditorCanvasMixin,
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
        self.setWindowTitle("YASMIN Editor")
        self.showMaximized()

        self.plugin_manager = manager
        self.root_model = StateMachine(name="")
        self.state_nodes: Dict[str, StateNode | ContainerStateNode] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self._blackboard_keys: List[Dict[str, str]] = []
        self._blackboard_key_metadata: Dict[str, Dict[str, str]] = {}
        self._highlight_blackboard_usage = True

        self.layout_seed = 42
        self.layout_rng = random.Random(self.layout_seed)
        self.current_container_path = [self.root_model]
        self.current_runtime_container_path: List[str] = []
        self.current_file_path: Optional[str] = None
        self.runtime_snapshot_file_path: Optional[str] = None
        self.runtime_mode_enabled = False
        self.runtime_auto_follow_enabled = False
        self.runtime_active_path: tuple[str, ...] = tuple()
        self.runtime_last_transition: Optional[
            tuple[tuple[str, ...], tuple[str, ...], str]
        ] = None

        self.runtime = None
        self.runtime_shell = None
        self._create_runtime()

        self.model_adapter = EditorModelAdapter(self)
        self.create_ui()

        self.statusBar().showMessage("Loading plugins...")
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    def _apply_theme(self) -> None:
        """Apply the configured application palette and stylesheet."""
        app = QApplication.instance()
        if app is None:
            return
        app.setPalette(build_qt_palette(PALETTE))
        app.setStyleSheet(build_stylesheet(PALETTE))

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
        self._shutdown_runtime_shell()
        self._destroy_runtime()
        self._delete_runtime_snapshot()

        self.canvas.clear_pending_placement()
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()

        event.accept()
        QApplication.quit()
        os._exit(0)
