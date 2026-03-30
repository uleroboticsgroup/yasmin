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
import tempfile
from html import escape
from typing import Dict, List, Optional, Set

from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QBrush, QCloseEvent, QColor, QPen
from PyQt5.QtWidgets import (QAbstractItemView, QAction, QApplication,
                             QComboBox, QDialog, QFileDialog, QFrame,
                             QHBoxLayout, QInputDialog, QLabel, QLineEdit,
                             QListWidget, QListWidgetItem, QMainWindow,
                             QMessageBox, QPushButton, QSplitter, QTextBrowser,
                             QToolBar, QVBoxLayout, QWidget)
from yasmin_editor.editor_gui.colors import (PALETTE, build_qt_palette,
                                             build_stylesheet)
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.dialogs.blackboard_key_dialog import \
    BlackboardKeyDialog
from yasmin_editor.editor_gui.dialogs.concurrence_dialog import \
    ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.outcome_description_dialog import \
    OutcomeDescriptionDialog
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import \
    StateMachineDialog
from yasmin_editor.editor_gui.dialogs.state_properties_dialog import \
    StatePropertiesDialog
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.model_adapter import EditorModelAdapter
from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import validate_model
from yasmin_editor.runtime import Runtime

from yasmin_plugins_manager.plugin_manager import PluginInfo, PluginManager


class YasminEditor(QMainWindow):
    """Main editor window for YASMIN state machines.

    Provides a graphical interface for creating, editing, and managing
    hierarchical state machines with support for Python, C++, and XML states.
    """

    def __init__(self, manager: PluginManager) -> None:
        """Initialize the YASMIN Editor.

        Args:
            manager: The PluginManager instance for handling plugins.
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
        self.runtime_active_path: tuple[str, ...] = tuple()
        self.runtime_last_transition: Optional[
            tuple[tuple[str, ...], tuple[str, ...], str]
        ] = None

        self.runtime = None
        self._create_runtime()

        self.model_adapter = EditorModelAdapter(self)
        self.create_ui()

        self.statusBar().showMessage("Loading plugins...")
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    def _apply_theme(self) -> None:
        app = QApplication.instance()
        if app is None:
            return
        app.setPalette(build_qt_palette(PALETTE))
        app.setStyleSheet(build_stylesheet(PALETTE))

    def _connect_runtime_signals(self) -> None:
        if self.runtime is None:
            return
        self.runtime.ready_changed.connect(self.update_runtime_actions)
        self.runtime.running_changed.connect(self.update_runtime_actions)
        self.runtime.blocked_changed.connect(self.update_runtime_actions)
        self.runtime.active_state_changed.connect(self.on_runtime_active_state_changed)
        self.runtime.active_transition_changed.connect(
            self.on_runtime_transition_changed
        )
        self.runtime.outcome_changed.connect(self.on_runtime_outcome_changed)
        self.runtime.status_changed.connect(self.on_runtime_status_changed)
        self.runtime.error_occurred.connect(self.on_runtime_error)
        self.runtime.log_cleared.connect(self.clear_runtime_log_view)
        self.runtime.log_appended.connect(self.append_runtime_log)

    def _create_runtime(self) -> None:
        self.runtime = Runtime()
        self._connect_runtime_signals()

    def _destroy_runtime(self) -> None:
        runtime = self.runtime
        self.runtime = None
        if runtime is None:
            return
        try:
            runtime.disconnect()
        except Exception:
            pass
        try:
            runtime.shutdown()
        except Exception:
            pass
        try:
            runtime.deleteLater()
        except Exception:
            pass
        del runtime

    def _recreate_runtime(self) -> None:
        self._destroy_runtime()
        self._create_runtime()

    def _delete_runtime_snapshot(self) -> None:
        if self.runtime_snapshot_file_path and os.path.isfile(
            self.runtime_snapshot_file_path
        ):
            try:
                os.remove(self.runtime_snapshot_file_path)
            except OSError:
                pass
        self.runtime_snapshot_file_path = None

    def _runtime_ready(self) -> bool:
        return self.runtime is not None

    def _get_live_runtime_active_path(self) -> tuple[str, ...]:
        if self.runtime is None or not self.runtime_mode_enabled:
            return tuple()
        try:
            active_path = tuple(self.runtime.get_active_path() or tuple())
        except Exception:
            active_path = tuple()
        self.runtime_active_path = active_path
        return active_path

    def _get_live_runtime_transition(
        self,
    ) -> Optional[tuple[tuple[str, ...], tuple[str, ...], str]]:
        if self.runtime is None or not self.runtime_mode_enabled:
            return None
        try:
            transition = self.runtime.get_last_transition()
        except Exception:
            transition = None
        self.runtime_last_transition = transition
        return transition

    def _schedule_runtime_highlight_refresh(self) -> None:
        if not self.runtime_mode_enabled:
            return
        QTimer.singleShot(0, self._refresh_runtime_highlighting_from_runtime)

    def _refresh_runtime_highlighting_from_runtime(self) -> None:
        if not self.runtime_mode_enabled:
            return
        self._get_live_runtime_active_path()
        self._get_live_runtime_transition()
        self.refresh_visual_highlighting()

    @property
    def root_sm_name(self) -> str:
        return self.root_model.name

    @root_sm_name.setter
    def root_sm_name(self, value: str) -> None:
        self.root_model.name = value

    @property
    def start_state(self) -> Optional[str]:
        return self.root_model.start_state

    @start_state.setter
    def start_state(self, value: Optional[str]) -> None:
        self.root_model.start_state = value

    def find_selected_item(self, item_type):
        for item in self.canvas.scene.selectedItems():
            if isinstance(item, item_type):
                return item
        return None

    def find_selected_state_node(self) -> Optional[StateNode | ContainerStateNode]:
        return self.find_selected_item((StateNode, ContainerStateNode))

    def find_selected_container(self) -> Optional[ContainerStateNode]:
        return self.find_selected_item(ContainerStateNode)

    def get_selected_container_or_warn(
        self, message: str
    ) -> Optional[ContainerStateNode]:
        container = self.find_selected_container()
        if container is None:
            QMessageBox.warning(self, "Error", message)
        return container

    def delete_connection_item(self, connection: ConnectionLine) -> None:
        self.unregister_connection_in_model(connection)
        connection.from_node.remove_connection(connection)
        connection.to_node.remove_connection(connection)
        self.canvas.scene.removeItem(connection)
        self.canvas.scene.removeItem(connection.arrow_head)
        self.canvas.scene.removeItem(connection.label_bg)
        self.canvas.scene.removeItem(connection.label)
        if connection in self.connections:
            self.connections.remove(connection)
        for existing_connection in self.connections:
            existing_connection.update_position()
        self.refresh_connection_port_visibility()

    def iter_state_subtree_items(self, state_node: StateNode | ContainerStateNode):
        yield state_node
        if isinstance(state_node, ContainerStateNode):
            for child_state in state_node.child_states.values():
                yield from self.iter_state_subtree_items(child_state)
            for final_outcome in state_node.final_outcomes.values():
                yield final_outcome

    def closeEvent(self, event: QCloseEvent) -> None:
        """Handle window close event to ensure proper cleanup."""
        self._destroy_runtime()
        self._delete_runtime_snapshot()

        # Clear all references
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()

        # Accept the close event
        event.accept()

        # Quit the application and exit the process
        QApplication.quit()

        # Force process termination
        os._exit(0)

    def populate_plugin_lists(self) -> None:
        """Populate the plugin lists with available Python, C++, and XML states."""
        for plugin in self.plugin_manager.python_plugins:
            item = QListWidgetItem(f"{plugin.module}.{plugin.class_name}")
            item.setData(Qt.UserRole, plugin)
            self.python_list.addItem(item)

        for plugin in self.plugin_manager.cpp_plugins:
            item = QListWidgetItem(plugin.class_name)
            item.setData(Qt.UserRole, plugin)
            self.cpp_list.addItem(item)

        for xml_plugin in self.plugin_manager.xml_files:
            display_name = f"{xml_plugin.package_name}/{xml_plugin.file_name}"
            item = QListWidgetItem(display_name)
            item.setData(Qt.UserRole, xml_plugin)
            self.xml_list.addItem(item)

    def filter_list(self, list_widget: QListWidget, text: str) -> None:
        """Filter a list widget based on search text."""
        for i in range(list_widget.count()):
            item = list_widget.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_blackboard_keys(self, text: str) -> None:
        """Filter blackboard keys based on search text."""
        for i in range(self.blackboard_list.count()):
            item = self.blackboard_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def format_blackboard_key_label(self, key_data: Dict[str, str]) -> str:
        label = f"{key_data.get('name', '')} ({key_data.get('key_type', 'in')})"

        default_type = str(key_data.get("default_type", "")).strip()
        if default_type:
            default_value = str(key_data.get("default_value", ""))
            label += f" [default: {default_value}, type: {default_type}]"

        return label

    def dicts_to_keys(self, keys: List[Dict[str, str]]) -> List[Key]:
        result: List[Key] = []
        for key_data in keys:
            key_name = str(key_data.get("name", "") or "").strip()
            if not key_name:
                continue
            result.append(
                Key(
                    name=key_name,
                    key_type=str(key_data.get("key_type", "in") or "in").strip(),
                    description=str(key_data.get("description", "") or "").strip(),
                    default_type=str(key_data.get("default_type", "") or "").strip(),
                    default_value=str(key_data.get("default_value", "") or ""),
                )
            )
        return result

    def keys_to_dicts(self, keys: List[Key]) -> List[Dict[str, str]]:
        result: List[Dict[str, str]] = []
        for key in keys:
            result.append(
                {
                    "name": key.name,
                    "key_type": key.key_type,
                    "description": key.description,
                    "default_type": key.default_type,
                    "default_value": (
                        "" if key.default_value is None else str(key.default_value)
                    ),
                }
            )
        return result

    def _get_plugin_key_usage(
        self, state_node: StateNode, key_info: Dict[str, str], is_input: bool
    ) -> Optional[Dict[str, str]]:
        key_name = str(key_info.get("name", "")).strip()
        if not key_name:
            return None

        effective_name = self.get_effective_blackboard_key_name(state_node, key_name)
        if not effective_name:
            return None

        description = str(key_info.get("description", "") or "").strip()
        return {
            "name": effective_name,
            "usage": "input" if is_input else "output",
            "description": description,
        }

    def _collect_blackboard_key_usage(self) -> Dict[str, Dict[str, str]]:
        usage_map: Dict[str, Dict[str, object]] = {}

        for state_node in self.state_nodes.values():
            plugin_info = getattr(state_node, "plugin_info", None)
            if plugin_info is None:
                continue

            for key_info in list(getattr(plugin_info, "input_keys", []) or []):
                usage = self._get_plugin_key_usage(state_node, key_info, True)
                if usage is None:
                    continue
                entry = usage_map.setdefault(
                    usage["name"],
                    {
                        "input": False,
                        "output": False,
                        "description": "",
                    },
                )
                entry["input"] = True
                if not entry["description"] and usage["description"]:
                    entry["description"] = usage["description"]

            for key_info in list(getattr(plugin_info, "output_keys", []) or []):
                usage = self._get_plugin_key_usage(state_node, key_info, False)
                if usage is None:
                    continue
                entry = usage_map.setdefault(
                    usage["name"],
                    {
                        "input": False,
                        "output": False,
                        "description": "",
                    },
                )
                entry["output"] = True
                if not entry["description"] and usage["description"]:
                    entry["description"] = usage["description"]

        derived_keys: Dict[str, Dict[str, str]] = {}
        for key_name, usage in usage_map.items():
            metadata = dict(self._blackboard_key_metadata.get(key_name, {}))
            if usage["input"] and usage["output"]:
                key_type = "in/out"
            elif usage["output"]:
                key_type = "out"
            else:
                key_type = "in"

            description = str(metadata.get("description", "") or "").strip()
            if not description:
                description = str(usage.get("description", "") or "").strip()

            default_type = ""
            default_value = ""
            if key_type in ("in", "in/out"):
                default_type = str(metadata.get("default_type", "") or "")
                if default_type:
                    default_value = str(metadata.get("default_value", "") or "")

            derived_keys[key_name] = {
                "name": key_name,
                "key_type": key_type,
                "description": description,
                "default_type": default_type,
                "default_value": default_value,
            }

        return dict(sorted(derived_keys.items(), key=lambda item: item[0].lower()))

    def get_selected_blackboard_key_name(self) -> Optional[str]:
        item = self.blackboard_list.currentItem()
        if item is None:
            return None
        key_data = item.data(Qt.UserRole) or {}
        return key_data.get("name")

    def add_root_default_row(self) -> None:
        pass

    def remove_root_default_row(self) -> None:
        pass

    def get_root_defaults(self) -> list:
        return []

    def on_blackboard_selection_changed(self) -> None:
        self.update_blackboard_usage_highlighting()

    def toggle_blackboard_highlighting(self, enabled: bool) -> None:
        self._highlight_blackboard_usage = enabled
        self.highlight_blackboard_btn.setText(
            "Highlight: On" if enabled else "Highlight: Off"
        )
        self.update_blackboard_usage_highlighting()

    def get_effective_blackboard_key_name(self, state_node, key_name: str) -> str:
        effective_key_name = key_name

        remap_chain = []
        current_node = state_node
        while current_node is not None:
            remap_chain.append(getattr(current_node, "remappings", {}) or {})
            current_node = getattr(current_node, "parent_container", None)

        for remappings in remap_chain:
            effective_key_name = remappings.get(effective_key_name, effective_key_name)

        return effective_key_name

    def _remove_state_node_entries(
        self, state_node: StateNode, prefix: str = ""
    ) -> None:
        full_name = f"{prefix}.{state_node.name}" if prefix else state_node.name
        if isinstance(state_node, ContainerStateNode):
            for child_state in list(state_node.child_states.values()):
                self._remove_state_node_entries(child_state, full_name)
        if full_name in self.state_nodes:
            del self.state_nodes[full_name]

    def _is_deleted_graphics_item(self, item) -> bool:
        if item is None:
            return True
        try:
            item.scene()
        except RuntimeError:
            return True
        return False

    def _is_deleted_connection_item(self, connection: Optional[ConnectionLine]) -> bool:
        if connection is None:
            return True
        try:
            connection.scene()
            connection.arrow_head.scene()
            connection.label_bg.scene()
            connection.label.scene()
        except RuntimeError:
            return True
        return False

    def apply_default_connection_visual_state(self, connection: ConnectionLine) -> None:
        if self._is_deleted_connection_item(connection):
            return

        try:
            is_selected = connection.isSelected()
        except RuntimeError:
            return

        try:
            connection.label_bg.setBrush(QBrush(PALETTE.connection_label_bg))
            connection._update_label_style(is_selected)
        except RuntimeError:
            return

    def apply_default_visual_state(self, item) -> None:
        if self._is_deleted_graphics_item(item):
            return

        try:
            is_selected = item.isSelected() if hasattr(item, "isSelected") else False
        except RuntimeError:
            return

        try:
            if isinstance(item, StateNode):
                item.setBrush(
                    QBrush(
                        PALETTE.state_fill(
                            item.plugin_info.plugin_type if item.plugin_info else None
                        )
                    )
                )
                item.setPen(
                    QPen(PALETTE.selection_pen, 4)
                    if is_selected
                    else QPen(PALETTE.state_pen, 3)
                )
            elif isinstance(item, ContainerStateNode):
                if getattr(item, "is_xml_reference", False):
                    item.setBrush(QBrush(PALETTE.container_xml_fill))
                    item.setPen(
                        QPen(PALETTE.selection_pen, 4)
                        if is_selected
                        else QPen(PALETTE.container_xml_pen, 3)
                    )
                elif item.is_concurrence:
                    item.setBrush(QBrush(PALETTE.container_concurrence_fill))
                    item.setPen(
                        QPen(PALETTE.selection_pen, 4)
                        if is_selected
                        else QPen(PALETTE.container_concurrence_pen, 3)
                    )
                else:
                    item.setBrush(QBrush(PALETTE.container_state_machine_fill))
                    item.setPen(
                        QPen(PALETTE.selection_pen, 4)
                        if is_selected
                        else QPen(PALETTE.container_state_machine_pen, 3)
                    )
            elif isinstance(item, FinalOutcomeNode):
                item.setBrush(QBrush(PALETTE.final_outcome_fill))
                item.setPen(
                    QPen(PALETTE.selection_pen, 4)
                    if is_selected
                    else QPen(PALETTE.final_outcome_pen, 3)
                )
        except RuntimeError:
            return

    def resolve_plugin_info_for_model(self, model: State) -> PluginInfo:
        if model.state_type == "py":
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.python_plugins
                    if item.module == model.module
                    and item.class_name == model.class_name
                ),
                None,
            )
        elif model.state_type == "cpp":
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.cpp_plugins
                    if item.class_name == model.class_name
                ),
                None,
            )
        else:
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.xml_files
                    if item.file_name == model.file_name
                    and (
                        not model.package_name
                        or item.package_name == model.package_name
                    )
                ),
                None,
            )
            if plugin is None and model.file_name:
                plugin = next(
                    (
                        item
                        for item in self.plugin_manager.xml_files
                        if item.file_name == model.file_name
                    ),
                    None,
                )

        if plugin is None:
            raise ValueError(f"Unable to resolve plugin for state '{model.name}'")
        return plugin

    def create_leaf_model(
        self,
        name: str,
        plugin_info: PluginInfo,
        description: str = "",
        remappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
    ) -> State:
        state_type = {"python": "py", "cpp": "cpp", "xml": "xml"}.get(
            plugin_info.plugin_type,
            plugin_info.plugin_type,
        )
        model = State(
            name=name,
            description=description or "",
            remappings=dict(remappings or {}),
            state_type=state_type,
            module=getattr(plugin_info, "module", None),
            class_name=getattr(plugin_info, "class_name", None),
            package_name=getattr(plugin_info, "package_name", None),
            file_name=getattr(plugin_info, "file_name", None),
        )
        for outcome_name in list(
            outcomes or getattr(plugin_info, "outcomes", []) or []
        ):
            model.add_outcome(Outcome(name=outcome_name))
        return model

    def create_container_model(
        self,
        name: str,
        is_concurrence: bool,
        outcomes: Optional[List[str]] = None,
        remappings: Optional[Dict[str, str]] = None,
        start_state: Optional[str] = None,
        default_outcome: Optional[str] = None,
        description: str = "",
    ) -> StateMachine | Concurrence:
        if is_concurrence:
            model: StateMachine | Concurrence = Concurrence(
                name=name,
                description=description or "",
                default_outcome=default_outcome,
                remappings=dict(remappings or {}),
            )
        else:
            model = StateMachine(
                name=name,
                description=description or "",
                start_state=start_state,
                remappings=dict(remappings or {}),
            )
        for outcome_name in outcomes or []:
            model.add_outcome(Outcome(name=outcome_name))
        return model

    def _create_connection_view(
        self, from_node, to_node, outcome: str
    ) -> ConnectionLine:
        connection = ConnectionLine(from_node, to_node, outcome)
        self.canvas.scene.addItem(connection)
        self.canvas.scene.addItem(connection.arrow_head)
        self.canvas.scene.addItem(connection.label_bg)
        self.canvas.scene.addItem(connection.label)
        self.connections.append(connection)
        for existing_connection in self.connections:
            existing_connection.update_position()
        return connection

    def _rename_state_node_entries(self, old_prefix: str, new_prefix: str) -> None:
        updates = {}
        for key, value in list(self.state_nodes.items()):
            if key == old_prefix or key.startswith(old_prefix + "."):
                suffix = key[len(old_prefix) :]
                updates[new_prefix + suffix] = value
                del self.state_nodes[key]
        self.state_nodes.update(updates)

    def _rename_state_node(self, state_node, new_name: str) -> None:
        old_name = state_node.name
        parent_container = getattr(state_node, "parent_container", None)
        parent_model = (
            self.root_model if parent_container is None else parent_container.model
        )
        old_prefix = self.get_state_node_key(old_name, parent_container)
        new_prefix = self.get_state_node_key(new_name, parent_container)

        parent_model.rename_state(old_name, new_name)

        if parent_container is not None:
            parent_container.child_states[new_name] = parent_container.child_states.pop(
                old_name
            )

        self._rename_state_node_entries(old_prefix, new_prefix)
        state_node.name = new_name
        self.update_start_state_combo()

    def apply_common_state_updates(
        self,
        state_node: StateNode | ContainerStateNode,
        new_name: str,
        remappings: Dict[str, str],
        description: str,
        defaults: List[Dict[str, str]],
    ) -> bool:
        if new_name != state_node.name:
            if self.has_state_name_conflict(
                new_name,
                getattr(state_node, "parent_container", None),
            ):
                QMessageBox.warning(
                    self, "Error", f"State '{new_name}' already exists!"
                )
                return False
            self._rename_state_node(state_node, new_name)

        state_node.remappings = remappings
        state_node.description = description
        state_node.defaults = defaults
        return True

    def create_state_node(
        self,
        name: str,
        plugin_info: PluginInfo,
        is_state_machine: bool = False,
        is_concurrence: bool = False,
        outcomes: List[str] = None,
        remappings: Dict[str, str] = None,
        start_state: str = None,
        default_outcome: str = None,
        description: str = "",
        defaults: List[Dict[str, str]] = None,
    ) -> None:
        """Create a new state node in the canvas."""
        if not name:
            QMessageBox.warning(self, "Validation Error", "Name is required!")
            return

        if self.has_state_name_conflict(name):
            QMessageBox.warning(self, "Error", f"State '{name}' already exists!")
            return

        if is_state_machine or is_concurrence:
            model = self.create_container_model(
                name=name,
                is_concurrence=is_concurrence,
                outcomes=outcomes,
                remappings=remappings,
                start_state=start_state,
                default_outcome=default_outcome,
                description=description,
            )
        else:
            model = self.create_leaf_model(
                name=name,
                plugin_info=plugin_info,
                description=description,
                remappings=remappings,
                outcomes=outcomes,
            )

        self.add_model_state(model, defaults=defaults)
        self.statusBar().showMessage(f"Added state: {name}", 2000)

    def add_container(self, is_concurrence: bool = False) -> None:
        """Add a new container (State Machine or Concurrence)."""
        dialog = (
            ConcurrenceDialog(parent=self)
            if is_concurrence
            else StateMachineDialog(parent=self)
        )
        if dialog.exec_():
            result = (
                dialog.get_concurrence_data()
                if is_concurrence
                else dialog.get_state_machine_data()
            )
            if result:
                name, outcomes, param, remappings, description, defaults = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=not is_concurrence,
                    is_concurrence=is_concurrence,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=param if not is_concurrence else None,
                    default_outcome=param if is_concurrence else None,
                    description=description,
                    defaults=defaults,
                )

    def show_help(self) -> None:
        """Display help dialog with usage instructions."""
        help_text = """
        <h2>YASMIN Editor - Quick Guide</h2>
        <h3>File Operations</h3>
        <b>New/Open/Save:</b> Create, load, or save state machines from XML files.
        <h3>Building State Machines</h3>
        <b>State Machine Name:</b> Set root name.<br>
        <b>Start State:</b> Select initial state.<br>
        <b>Add State:</b> Add regular state (Python/C++/XML).<br>
        <b>Add State Machine:</b> Add nested container.<br>
        <b>Add Concurrence:</b> Add parallel container.<br>
        <b>Add Final Outcome:</b> Add exit point.
        <h3>Working with States</h3>
        <b>Double-click:</b> Plugin to add state.<br>
        <b>Right-click:</b> State options.<br>
        <b>Drag:</b> Reposition states.<br>
        <b>Delete Selected:</b> Remove items.
        <h3>Creating Transitions</h3>
        <b>Drag from blue port:</b> Create transitions.<br>
        <b>Select outcome:</b> Choose trigger.
        <h3>Containers</h3>
        <b>Nested States:</b> Double-click to edit.<br>
        <b>Final Outcomes:</b> Exit points.<br>
        <b>State Machine:</b> Sequential.<br>
        <b>Concurrence:</b> Parallel.
        <h3>Canvas Navigation</h3>
        <b>Scroll:</b> Zoom.<br>
        <b>Drag:</b> Pan.
        <h3>Validation</h3>
        • Name set<br>
        • Start state selected<br>
        • Final outcome exists<br>
        • Unique names
        <h3>Tips</h3>
        • Use filters to find states.<br>
        • Containers auto-resize.<br>
        • Concurrence states transition to internal final outcomes.<br>
        • XML SMs are regular states.
        """

        dialog = QDialog(self)
        dialog.setWindowTitle("YASMIN Editor Help")
        dialog.setMinimumSize(600, 500)
        dialog.setMaximumSize(800, 600)
        layout = QVBoxLayout(dialog)
        text_browser = QTextBrowser()
        text_browser.setHtml(help_text)
        text_browser.setOpenExternalLinks(False)
        layout.addWidget(text_browser)
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(dialog.accept)
        ok_button.setDefault(True)
        button_layout.addWidget(ok_button)
        dialog.exec_()

    def new_state_machine(self) -> bool:
        """Create a new state machine, clearing the current one."""
        if self.runtime_mode_enabled:
            self._show_read_only_message()
            return False

        reply = QMessageBox.question(
            self,
            "New State Machine",
            "Are you sure you want to create a new state machine? All unsaved changes will be lost.",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self.reset_editor_state()
            self.statusBar().showMessage("New state machine created", 2000)
            return True

        return False

    def open_state_machine(self) -> None:
        """Open a state machine from an XML file."""
        if self.runtime_mode_enabled:
            self._show_read_only_message()
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            try:
                if not self.new_state_machine():
                    return

                self.load_from_xml(file_path)
                self.statusBar().showMessage(f"Opened: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to open file: {str(e)}")

    @property
    def current_container_model(self):
        return self.current_container_path[-1]

    @property
    def current_parent_model(self):
        if len(self.current_container_path) < 2:
            return None
        return self.current_container_path[-2]

    @property
    def start_state(self) -> Optional[str]:
        model = self.current_container_model
        return model.start_state if isinstance(model, StateMachine) else None

    @start_state.setter
    def start_state(self, value: Optional[str]) -> None:
        model = self.current_container_model
        if isinstance(model, StateMachine):
            model.start_state = value

    @property
    def default_outcome(self) -> Optional[str]:
        model = self.current_container_model
        return model.default_outcome if isinstance(model, Concurrence) else None

    @default_outcome.setter
    def default_outcome(self, value: Optional[str]) -> None:
        model = self.current_container_model
        if isinstance(model, Concurrence):
            model.default_outcome = value

    def clear_current_scene(self) -> None:
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()

    def sync_current_container_layout(self) -> None:
        container_model = self.current_container_model
        for state_name, state_view in self.state_nodes.items():
            container_model.layout.set_state_position(
                state_name,
                float(state_view.pos().x()),
                float(state_view.pos().y()),
            )
        for outcome_name, outcome_view in self.final_outcomes.items():
            container_model.layout.set_outcome_position(
                outcome_name,
                float(outcome_view.pos().x()),
                float(outcome_view.pos().y()),
            )

    def update_container_controls(self) -> None:
        if not hasattr(self, "root_sm_name_edit"):
            return
        model = self.current_container_model
        self.root_sm_name_edit.blockSignals(True)
        self.root_sm_description_edit.blockSignals(True)
        self.start_state_combo.blockSignals(True)

        if isinstance(model, StateMachine):
            self.root_sm_name_label.setText("<b>State Machine Name:</b>")
            self.start_state_label.setText("<b>Start State:</b>")
        else:
            self.root_sm_name_label.setText("<b>Concurrence Name:</b>")
            self.start_state_label.setText("<b>Default Outcome:</b>")

        self.root_sm_name_edit.setText(model.name)
        self.root_sm_description_edit.setText(model.description)

        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")
        if isinstance(model, StateMachine):
            for state_name in sorted(model.states.keys()):
                self.start_state_combo.addItem(state_name)
            current_value = model.start_state
        else:
            for outcome in model.outcomes:
                self.start_state_combo.addItem(outcome.name)
            current_value = model.default_outcome

        if current_value:
            index = self.start_state_combo.findText(current_value)
            self.start_state_combo.setCurrentIndex(index if index >= 0 else 0)
        else:
            self.start_state_combo.setCurrentIndex(0)

        self.root_sm_name_edit.blockSignals(False)
        self.root_sm_description_edit.blockSignals(False)
        self.start_state_combo.blockSignals(False)

    def on_root_sm_name_changed(self, text: str) -> None:
        model = self.current_container_model
        parent_model = self.current_parent_model
        text = text.strip()
        if parent_model is None:
            old_name = model.name
            model.name = text
            if isinstance(model, StateMachine):
                model.rename_transition_owner(old_name, text)
        else:
            old_name = model.name
            if text != old_name and text in parent_model.states:
                return
            parent_model.rename_state(old_name, text)
        self.refresh_breadcrumbs()

    def on_root_sm_description_changed(self, text: str) -> None:
        self.current_container_model.description = text

    def on_start_state_changed(self, text: str) -> None:
        if isinstance(self.current_container_model, StateMachine):
            self.start_state = None if text == "(None)" else text
        else:
            self.default_outcome = None if text == "(None)" else text

    def update_start_state_combo(self) -> None:
        self.update_container_controls()

    def reset_editor_state(self, model: Optional[StateMachine] = None) -> None:
        self.clear_current_scene()
        self._blackboard_keys = []
        self._blackboard_key_metadata = {}
        self.root_model = model or self.model_adapter.create_empty_root_model()
        self.current_container_path = [self.root_model]
        self.current_file_path = None
        self.current_runtime_container_path = []
        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self._delete_runtime_snapshot()
        self.clear_runtime_log_view()
        self.refresh_blackboard_keys_list()
        self.update_container_controls()
        self.refresh_breadcrumbs()

    def save_to_xml(self, file_path: str) -> None:
        self.model_adapter.save_to_xml(file_path)

    def get_root_state_nodes(self) -> Dict[str, StateNode | ContainerStateNode]:
        return self.state_nodes

    def get_container_path(self, container: Optional[ContainerStateNode]) -> str:
        if container is None:
            return ""
        return container.name

    def get_state_node_key(
        self,
        name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        return name

    def register_state_node(
        self,
        state_node: StateNode | ContainerStateNode,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        self.state_nodes[state_node.name] = state_node
        return state_node.name

    def find_target_view(self, target_name: str, source_container=None):
        target = self.state_nodes.get(target_name)
        if target is not None:
            return target
        return self.final_outcomes.get(target_name)

    def has_state_name_conflict(
        self,
        state_name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> bool:
        return state_name in self.current_container_model.states

    def add_model_state(
        self,
        model: State,
        defaults: Optional[List[Dict[str, str]]] = None,
        parent_container: Optional[ContainerStateNode] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> StateNode | ContainerStateNode:
        position = self.get_free_position()
        x = position.x() if x is None else x
        y = position.y() if y is None else y

        container_model = self.current_container_model
        container_model.add_state(model)
        container_model.layout.set_state_position(model.name, x, y)
        node = self.model_adapter.create_state_view(model, x=x, y=y)
        node.defaults = defaults or []

        if isinstance(container_model, StateMachine):
            if len(container_model.states) == 1 and not container_model.start_state:
                container_model.start_state = model.name
        self.update_start_state_combo()
        self.sync_blackboard_keys()
        self.refresh_connection_port_visibility()
        return node

    def register_connection_in_model(self, from_node, to_node, outcome: str) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.set_outcome_rule(to_node.name, from_node.name, outcome)
            return
        owner_model.add_transition(
            from_node.name,
            Transition(source_outcome=outcome, target=to_node.name),
        )

    def unregister_connection_in_model(self, connection: ConnectionLine) -> None:
        owner_model = self.current_container_model
        if isinstance(owner_model, Concurrence):
            owner_model.remove_outcome_rule(
                connection.to_node.name, connection.from_node.name
            )
            return
        owner_model.remove_transition(
            connection.from_node.name,
            connection.outcome,
            connection.to_node.name,
        )

    def delete_state_item(self, state_node: StateNode | ContainerStateNode) -> None:
        for connection in list(state_node.connections):
            self.delete_connection_item(connection)

        self.state_nodes.pop(state_node.name, None)
        self.current_container_model.remove_state(state_node.name)
        self.canvas.scene.removeItem(state_node)
        self.update_start_state_combo()
        self.sync_blackboard_keys()
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(f"Deleted state: {state_node.name}", 2000)

    def delete_final_outcome_item(self, outcome_node: FinalOutcomeNode) -> None:
        for connection in list(outcome_node.connections):
            self.delete_connection_item(connection)

        self.final_outcomes.pop(outcome_node.name, None)
        self.current_container_model.remove_outcome(outcome_node.name)
        self.canvas.scene.removeItem(outcome_node)
        self.update_start_state_combo()
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(
            f"Deleted final outcome: {outcome_node.name}",
            2000,
        )

    def add_state_to_container(self) -> None:
        self.add_state()

    def add_state_machine_to_container(self) -> None:
        self.add_state_machine()

    def add_concurrence_to_container(self) -> None:
        self.add_concurrence()

    def _get_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
        metadata: Dict[str, Dict[str, str]] = {}
        for key in getattr(container_model, "keys", []) or []:
            key_name = str(getattr(key, "name", "") or "").strip()
            if not key_name:
                continue
            metadata[key_name] = {
                "description": str(getattr(key, "description", "") or "").strip(),
                "key_type": str(getattr(key, "key_type", "in") or "in").strip(),
                "default_type": str(getattr(key, "default_type", "") or "").strip(),
                "default_value": (
                    ""
                    if getattr(key, "default_value", None) is None
                    else str(getattr(key, "default_value", ""))
                ),
            }
        return metadata

    def _set_container_metadata_map(
        self,
        container_model: StateMachine | Concurrence,
        metadata: Dict[str, Dict[str, str]],
    ) -> None:
        container_model.keys = self.dicts_to_keys(
            [
                {
                    "name": key_name,
                    "key_type": values.get("key_type", "in"),
                    "description": values.get("description", ""),
                    "default_type": values.get("default_type", ""),
                    "default_value": values.get("default_value", ""),
                }
                for key_name, values in sorted(
                    metadata.items(), key=lambda item: item[0].lower()
                )
            ]
        )

    def _collect_blackboard_key_usage_for_model(
        self,
        container_model: StateMachine | Concurrence,
    ) -> tuple[Dict[str, Dict[str, str]], set[str]]:
        usage_map: Dict[str, Dict[str, object]] = {}
        metadata_map = self._get_container_metadata_map(container_model)
        hidden_key_names: set[str] = set()

        def add_usage(key_name: str, usage_kind: str, description: str) -> None:
            entry = usage_map.setdefault(
                key_name,
                {
                    "input": False,
                    "output": False,
                    "description": "",
                },
            )
            entry[usage_kind] = True
            if not entry["description"] and description:
                entry["description"] = description

        def resolve_local_name(
            raw_name: str,
            remap_chain: List[Dict[str, str]],
        ) -> str:
            effective_name = raw_name
            intermediate_names: list[str] = []

            for remappings in remap_chain:
                intermediate_names.append(effective_name)
                effective_name = remappings.get(effective_name, effective_name)

            hidden_key_names.update(
                name for name in intermediate_names if name and name != effective_name
            )
            return effective_name

        def visit_state(
            state_model: State, ancestor_remaps: List[Dict[str, str]]
        ) -> None:
            state_remaps = [dict(state_model.remappings)] + ancestor_remaps

            if isinstance(state_model, (StateMachine, Concurrence)):
                for child_state in state_model.states.values():
                    visit_state(child_state, state_remaps)
                return

            try:
                plugin_info = self.resolve_plugin_info_for_model(state_model)
            except Exception:
                return

            for key_info in list(getattr(plugin_info, "input_keys", []) or []):
                raw_name = str(key_info.get("name", "")).strip()
                if not raw_name:
                    continue
                add_usage(
                    resolve_local_name(raw_name, state_remaps),
                    "input",
                    str(key_info.get("description", "") or "").strip(),
                )

            for key_info in list(getattr(plugin_info, "output_keys", []) or []):
                raw_name = str(key_info.get("name", "")).strip()
                if not raw_name:
                    continue
                add_usage(
                    resolve_local_name(raw_name, state_remaps),
                    "output",
                    str(key_info.get("description", "") or "").strip(),
                )

        for child_state in container_model.states.values():
            visit_state(child_state, [])

        derived_keys: Dict[str, Dict[str, str]] = {}
        for key_name, usage in usage_map.items():
            metadata = dict(metadata_map.get(key_name, {}))
            if usage["input"] and usage["output"]:
                key_type = "in/out"
            elif usage["output"]:
                key_type = "out"
            else:
                key_type = "in"

            description = str(metadata.get("description", "") or "").strip()
            if not description:
                description = str(usage.get("description", "") or "").strip()

            default_type = ""
            default_value = ""
            if key_type in ("in", "in/out"):
                default_type = str(metadata.get("default_type", "") or "")
                if default_type:
                    default_value = str(metadata.get("default_value", "") or "")

            derived_keys[key_name] = {
                "name": key_name,
                "key_type": key_type,
                "description": description,
                "default_type": default_type,
                "default_value": default_value,
            }

        return (
            dict(sorted(derived_keys.items(), key=lambda item: item[0].lower())),
            hidden_key_names,
        )

    def _has_persistent_blackboard_metadata(
        self,
        metadata: Dict[str, str],
    ) -> bool:
        return any(
            str(metadata.get(field, "") or "").strip()
            for field in ["default_type", "default_value"]
        )

    def _merge_container_keys(
        self,
        container_model: StateMachine | Concurrence,
    ) -> Dict[str, Dict[str, str]]:
        derived_keys, hidden_key_names = self._collect_blackboard_key_usage_for_model(
            container_model
        )
        metadata_map = self._get_container_metadata_map(container_model)
        merged: Dict[str, Dict[str, str]] = {}

        for key_name in sorted(
            set(derived_keys.keys()) | set(metadata_map.keys()), key=str.lower
        ):
            metadata = dict(metadata_map.get(key_name, {}))
            if key_name not in derived_keys:
                if key_name in hidden_key_names:
                    continue
                if not self._has_persistent_blackboard_metadata(metadata):
                    continue

            derived = dict(derived_keys.get(key_name, {}))
            key_type = str(
                derived.get("key_type", metadata.get("key_type", "in")) or "in"
            )
            merged[key_name] = {
                "name": key_name,
                "key_type": key_type,
                "description": str(
                    metadata.get("description", "")
                    or derived.get("description", "")
                    or ""
                ),
                "default_type": str(
                    metadata.get("default_type", "")
                    or derived.get("default_type", "")
                    or ""
                ),
                "default_value": str(
                    metadata.get("default_value", "")
                    or derived.get("default_value", "")
                    or ""
                ),
            }
        return merged

    def _rebuild_root_blackboard_keys(self) -> None:
        merged_root = self._merge_container_keys(self.root_model)
        self.root_model.keys = self.dicts_to_keys(list(merged_root.values()))

    def sync_blackboard_keys(self) -> None:
        current_container = self.current_container_model
        merged = self._merge_container_keys(current_container)
        self._set_container_metadata_map(current_container, merged)
        self._blackboard_keys = list(merged.values())
        self._blackboard_key_metadata = {
            item["name"]: {
                "description": item.get("description", ""),
                "key_type": item.get("key_type", "in"),
                "default_type": item.get("default_type", ""),
                "default_value": item.get("default_value", ""),
            }
            for item in self._blackboard_keys
        }
        self._rebuild_root_blackboard_keys()
        self.refresh_blackboard_keys_list()

    def refresh_blackboard_keys_list(self) -> None:
        current_key_name = self.get_selected_blackboard_key_name()
        merged = self._merge_container_keys(self.current_container_model)
        self._blackboard_key_metadata = {
            key_name: {
                "description": values.get("description", ""),
                "key_type": values.get("key_type", "in"),
                "default_type": values.get("default_type", ""),
                "default_value": values.get("default_value", ""),
            }
            for key_name, values in merged.items()
        }
        self._blackboard_keys = list(merged.values())
        self.blackboard_list.clear()

        for key_data in sorted(
            self._blackboard_keys, key=lambda item: item.get("name", "").lower()
        ):
            item = QListWidgetItem(self.format_blackboard_key_label(key_data))
            item.setData(Qt.UserRole, dict(key_data))
            description = key_data.get("description", "")
            if description:
                item.setToolTip(description)
            self.blackboard_list.addItem(item)

        self.filter_blackboard_keys(self.blackboard_filter.text())

        if current_key_name:
            for i in range(self.blackboard_list.count()):
                item = self.blackboard_list.item(i)
                key_data = item.data(Qt.UserRole) or {}
                if key_data.get("name") == current_key_name:
                    self.blackboard_list.setCurrentItem(item)
                    break

        self.update_blackboard_usage_highlighting()

    def set_blackboard_keys(
        self, keys: List[Dict[str, str]], sync: bool = True
    ) -> None:

        self.root_model.keys = self.dicts_to_keys(keys)
        self._blackboard_key_metadata = self._get_container_metadata_map(
            self.current_container_model
        )
        self._blackboard_keys = [dict(item) for item in keys]
        if sync:
            self.sync_blackboard_keys()
        else:
            self.refresh_blackboard_keys_list()

    def get_blackboard_keys(self) -> List[Dict[str, str]]:
        self.sync_blackboard_keys()
        return self.keys_to_dicts(self.current_container_model.keys)

    def add_root_default_row_with_data(self, data: dict) -> None:
        key_name = str(data.get("key", "") or "").strip()
        if not key_name:
            return
        metadata = self._get_container_metadata_map(self.current_container_model)
        metadata[key_name] = {
            "description": str(data.get("description", "") or "").strip(),
            "key_type": "in",
            "default_type": str(data.get("type", "") or "").strip(),
            "default_value": str(data.get("value", "") or "").strip(),
        }
        self._set_container_metadata_map(self.current_container_model, metadata)
        self.sync_blackboard_keys()

    def save_state_machine(self) -> None:
        self.sync_current_container_layout()
        validation = validate_model(self.root_model)
        errors = [f"- {item.message}" for item in validation.errors]

        if errors:
            error_msg = (
                "Cannot save state machine. Please fix the following issues:\n\n"
                + "\n".join(errors)
            )
            reply = QMessageBox.critical(
                self,
                "Validation Errors",
                error_msg + "\n\nDo you want to save anyway?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply == QMessageBox.No:
                return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            if not file_path.lower().endswith(".xml"):
                file_path += ".xml"

            try:
                self.save_to_xml(file_path)
                self.current_file_path = file_path
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def create_ui(self) -> None:
        """Create and setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        toolbar = QToolBar()
        self.addToolBar(toolbar)

        self.new_action = QAction("New", self)
        self.new_action.setShortcut("Ctrl+N")
        self.new_action.triggered.connect(self.new_state_machine)
        toolbar.addAction(self.new_action)

        self.open_action = QAction("Open", self)
        self.open_action.setShortcut("Ctrl+O")
        self.open_action.triggered.connect(self.open_state_machine)
        toolbar.addAction(self.open_action)

        self.save_action = QAction("Save", self)
        self.save_action.setShortcut("Ctrl+S")
        self.save_action.triggered.connect(self.save_state_machine)
        toolbar.addAction(self.save_action)

        toolbar.addSeparator()

        self.add_state_action = QAction("Add State", self)
        self.add_state_action.triggered.connect(self.add_state)
        toolbar.addAction(self.add_state_action)

        self.add_state_machine_action = QAction("Add State Machine", self)
        self.add_state_machine_action.triggered.connect(self.add_state_machine)
        toolbar.addAction(self.add_state_machine_action)

        self.add_concurrence_action = QAction("Add Concurrence", self)
        self.add_concurrence_action.triggered.connect(self.add_concurrence)
        toolbar.addAction(self.add_concurrence_action)

        self.add_final_action = QAction("Add Final Outcome", self)
        self.add_final_action.triggered.connect(self.add_final_outcome)
        toolbar.addAction(self.add_final_action)

        toolbar.addSeparator()

        self.edit_current_action = QAction("Edit Current Container", self)
        self.edit_current_action.triggered.connect(self.edit_current_container)
        toolbar.addAction(self.edit_current_action)

        self.delete_action = QAction("Delete Selected", self)
        self.delete_action.triggered.connect(self.delete_selected)
        toolbar.addAction(self.delete_action)

        toolbar.addSeparator()

        help_action = QAction("Help", self)
        help_action.triggered.connect(self.show_help)
        toolbar.addAction(help_action)

        toolbar.addSeparator()

        self.runtime_mode_button = QPushButton("Runtime Mode")
        self.runtime_mode_button.setCheckable(True)
        self.runtime_mode_button.setToolTip(
            "Enter or leave runtime mode using the current state machine XML snapshot."
        )
        self.runtime_mode_button.clicked.connect(self.toggle_runtime_mode)
        toolbar.addWidget(self.runtime_mode_button)

        self.blackboard_widget = QWidget()
        blackboard_layout = QVBoxLayout(self.blackboard_widget)
        blackboard_layout.setContentsMargins(0, 0, 0, 0)
        blackboard_layout.addWidget(QLabel("<b>Blackboard Keys:</b>"))
        self.blackboard_filter = QLineEdit()
        self.blackboard_filter.setPlaceholderText("Filter blackboard keys...")
        self.blackboard_filter.textChanged.connect(self.filter_blackboard_keys)
        blackboard_layout.addWidget(self.blackboard_filter)
        self.blackboard_list = QListWidget()
        self.blackboard_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.blackboard_list.itemSelectionChanged.connect(
            self.on_blackboard_selection_changed
        )
        self.blackboard_list.itemDoubleClicked.connect(
            self.edit_selected_blackboard_key
        )
        blackboard_layout.addWidget(self.blackboard_list)
        blackboard_btn_row = QHBoxLayout()
        self.highlight_blackboard_btn = QPushButton("Highlight: On")
        self.highlight_blackboard_btn.setCheckable(True)
        self.highlight_blackboard_btn.setChecked(True)
        self.highlight_blackboard_btn.toggled.connect(
            self.toggle_blackboard_highlighting
        )
        blackboard_btn_row.addWidget(self.highlight_blackboard_btn)
        blackboard_layout.addLayout(blackboard_btn_row)
        left_layout.addWidget(self.blackboard_widget)

        self.editor_sidebar_widget = QWidget()
        editor_sidebar_layout = QVBoxLayout(self.editor_sidebar_widget)
        editor_sidebar_layout.setContentsMargins(0, 0, 0, 0)

        editor_sidebar_layout.addWidget(QLabel("<b>Python States:</b>"))
        self.python_filter = QLineEdit()
        self.python_filter.setPlaceholderText("Filter Python states...")
        self.python_filter.textChanged.connect(
            lambda value: self.filter_list(self.python_list, value)
        )
        editor_sidebar_layout.addWidget(self.python_filter)
        self.python_list = QListWidget()
        self.python_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        editor_sidebar_layout.addWidget(self.python_list)

        editor_sidebar_layout.addWidget(QLabel("<b>C++ States:</b>"))
        self.cpp_filter = QLineEdit()
        self.cpp_filter.setPlaceholderText("Filter C++ states...")
        self.cpp_filter.textChanged.connect(
            lambda value: self.filter_list(self.cpp_list, value)
        )
        editor_sidebar_layout.addWidget(self.cpp_filter)
        self.cpp_list = QListWidget()
        self.cpp_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        editor_sidebar_layout.addWidget(self.cpp_list)

        editor_sidebar_layout.addWidget(QLabel("<b>XML State Machines:</b>"))
        self.xml_filter = QLineEdit()
        self.xml_filter.setPlaceholderText("Filter XML state machines...")
        self.xml_filter.textChanged.connect(
            lambda value: self.filter_list(self.xml_list, value)
        )
        editor_sidebar_layout.addWidget(self.xml_filter)
        self.xml_list = QListWidget()
        self.xml_list.itemDoubleClicked.connect(self.on_xml_double_clicked)
        editor_sidebar_layout.addWidget(self.xml_list)
        left_layout.addWidget(self.editor_sidebar_widget)

        self.runtime_sidebar_widget = QWidget()
        runtime_sidebar_layout = QVBoxLayout(self.runtime_sidebar_widget)
        runtime_sidebar_layout.setContentsMargins(0, 0, 0, 0)
        runtime_sidebar_layout.addWidget(QLabel("<b>Logs:</b>"))
        self.runtime_log_view = QTextBrowser()
        self.runtime_log_view.setReadOnly(True)
        self.runtime_log_view.setOpenExternalLinks(False)
        self.runtime_log_view.setOpenLinks(False)
        self.runtime_log_view.setLineWrapMode(QTextBrowser.NoWrap)
        self.runtime_log_view.setProperty("viewerText", True)
        self.runtime_log_view.document().setDocumentMargin(8)
        runtime_sidebar_layout.addWidget(self.runtime_log_view)
        self.runtime_sidebar_widget.setVisible(False)
        left_layout.addWidget(self.runtime_sidebar_widget)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        metadata_widget = QWidget()
        metadata_layout = QVBoxLayout(metadata_widget)
        metadata_layout.setContentsMargins(0, 0, 0, 0)

        row1 = QHBoxLayout()
        self.root_sm_name_label = QLabel("<b>State Machine Name:</b>")
        row1.addWidget(self.root_sm_name_label)
        self.root_sm_name_edit = QLineEdit()
        self.root_sm_name_edit.setProperty("flatInput", True)
        self.root_sm_name_edit.setPlaceholderText("Enter container name...")
        self.root_sm_name_edit.textChanged.connect(self.on_root_sm_name_changed)
        row1.addWidget(self.root_sm_name_edit)

        self.start_state_label = QLabel("<b>Start State:</b>")
        row1.addWidget(self.start_state_label)
        self.start_state_combo = QComboBox()
        self.start_state_combo.setProperty("flatInput", True)
        self.start_state_combo.addItem("(None)")
        self.start_state_combo.currentTextChanged.connect(self.on_start_state_changed)
        row1.addWidget(self.start_state_combo)
        metadata_layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("<b>Description:</b>"))
        self.root_sm_description_edit = QLineEdit()
        self.root_sm_description_edit.setProperty("flatInput", True)
        self.root_sm_description_edit.setPlaceholderText(
            "Enter container description..."
        )
        self.root_sm_description_edit.textChanged.connect(
            self.on_root_sm_description_changed
        )
        row2.addWidget(self.root_sm_description_edit)
        metadata_layout.addLayout(row2)

        right_layout.addWidget(metadata_widget)

        self.canvas_header = QLabel(
            "<b>State Machine Canvas:</b> "
            "<i>(Ctrl + double-click a nested container to enter it, drag from blue port to create transitions, scroll to zoom, right-click for options)</i>"
        )
        right_layout.addWidget(self.canvas_header)

        breadcrumb_widget = QWidget()
        self.breadcrumb_layout = QHBoxLayout(breadcrumb_widget)
        self.breadcrumb_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(breadcrumb_widget)

        self.runtime_controls_widget = QWidget()
        self.runtime_controls_layout = QHBoxLayout(self.runtime_controls_widget)
        self.runtime_controls_layout.setContentsMargins(0, 0, 0, 0)

        self.runtime_status_label = QLabel("Ready")
        self.runtime_status_label.setAlignment(Qt.AlignCenter)
        self.runtime_status_label.setMinimumWidth(120)
        self.runtime_status_label.setTextFormat(Qt.PlainText)
        self.runtime_controls_layout.addWidget(self.runtime_status_label)

        self.runtime_play_button = QPushButton("Play")
        self.runtime_play_button.setToolTip(
            "Start the runtime or resume execution after a pause."
        )
        self.runtime_play_button.clicked.connect(self.on_runtime_play_clicked)
        self.runtime_controls_layout.addWidget(self.runtime_play_button)

        self.runtime_pause_button = QPushButton("Pause")
        self.runtime_pause_button.setToolTip(
            "Pause execution at the next state boundary."
        )
        self.runtime_pause_button.clicked.connect(self.on_runtime_pause_clicked)
        self.runtime_controls_layout.addWidget(self.runtime_pause_button)

        self.runtime_step_button = QPushButton("Play Once")
        self.runtime_step_button.setToolTip(
            "Execute exactly one state and pause before the following state starts."
        )
        self.runtime_step_button.clicked.connect(self.on_runtime_step_clicked)
        self.runtime_controls_layout.addWidget(self.runtime_step_button)

        self.runtime_cancel_state_button = QPushButton("Cancel State")
        self.runtime_cancel_state_button.setToolTip(
            "Request cancellation of the currently active state."
        )
        self.runtime_cancel_state_button.clicked.connect(
            self.on_runtime_cancel_state_clicked
        )
        self.runtime_controls_layout.addWidget(self.runtime_cancel_state_button)

        self.runtime_cancel_sm_button = QPushButton("Cancel State Machine")
        self.runtime_cancel_sm_button.setToolTip(
            "Request cancellation of the complete runtime state machine."
        )
        self.runtime_cancel_sm_button.clicked.connect(self.on_runtime_cancel_sm_clicked)
        self.runtime_controls_layout.addWidget(self.runtime_cancel_sm_button)

        self.runtime_restart_button = QPushButton("Restart")
        self.runtime_restart_button.setToolTip(
            "Recreate the runtime state machine from a fresh XML snapshot."
        )
        self.runtime_restart_button.clicked.connect(self.restart_runtime_mode)
        self.runtime_controls_layout.addWidget(self.runtime_restart_button)

        self.runtime_controls_layout.addStretch()
        right_layout.addWidget(self.runtime_controls_widget)

        self.canvas_frame = QFrame()
        self.canvas_frame_layout = QVBoxLayout(self.canvas_frame)
        self.canvas_frame_layout.setContentsMargins(0, 0, 0, 0)

        self.canvas = StateMachineCanvas()
        self.canvas.editor_ref = self
        self.canvas.scene.selectionChanged.connect(self.refresh_visual_highlighting)
        self.canvas_frame_layout.addWidget(self.canvas)
        right_layout.addWidget(self.canvas_frame)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 1000])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.statusBar()
        self.refresh_breadcrumbs()
        self.update_container_controls()
        self.update_runtime_actions()

    def fit_current_view(self) -> None:
        """Fit the currently rendered container into the canvas view."""
        if not hasattr(self, "canvas"):
            return
        items = self.canvas.scene.items()
        self.canvas.resetTransform()
        if not items:
            self.canvas.scene.setSceneRect(-400, -300, 800, 600)
            self.canvas.centerOn(0, 0)
            return

        bounds = self.canvas.scene.itemsBoundingRect()
        margin = 10.0
        padded = bounds.adjusted(-margin, -margin, margin, margin)
        self.canvas.scene.setSceneRect(padded)
        self.canvas.fitInView(padded, Qt.KeepAspectRatio)
        self.canvas.centerOn(bounds.center())

    def create_runtime_xml_snapshot(self) -> str:
        self.sync_current_container_layout()
        self._delete_runtime_snapshot()

        fd, temp_path = tempfile.mkstemp(
            prefix="yasmin_editor_runtime_",
            suffix=".xml",
        )
        os.close(fd)
        self.runtime_snapshot_file_path = temp_path

        self.save_to_xml(self.runtime_snapshot_file_path)
        return self.runtime_snapshot_file_path

    def _set_runtime_mode_button_checked(self, checked: bool) -> None:
        if not hasattr(self, "runtime_mode_button"):
            return
        self.runtime_mode_button.blockSignals(True)
        self.runtime_mode_button.setChecked(checked)
        self.runtime_mode_button.blockSignals(False)

    def _enter_runtime_mode(self) -> bool:
        self._recreate_runtime()

        try:
            runtime_path = self.create_runtime_xml_snapshot()
        except Exception as exc:
            self.runtime_mode_enabled = False
            self._set_runtime_mode_button_checked(False)
            QMessageBox.critical(
                self,
                "Runtime Error",
                f"Failed to enter runtime mode:\n{exc}",
            )
            self.update_runtime_actions()
            return False

        if self.runtime is None or not self.runtime.create_sm_from_file(runtime_path):
            self.runtime_mode_enabled = False
            self._set_runtime_mode_button_checked(False)
            self.update_runtime_actions()
            return False

        self.runtime_mode_enabled = True
        self._get_live_runtime_active_path()
        self._get_live_runtime_transition()
        self.render_current_container()
        self.set_canvas_runtime_visual_state()
        self.statusBar().showMessage("Runtime mode enabled", 3000)
        return True

    def on_runtime_play_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.play()

    def on_runtime_pause_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.pause()

    def on_runtime_step_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.play_once()

    def on_runtime_cancel_state_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.cancel_state()

    def on_runtime_cancel_sm_clicked(self) -> None:
        if self.runtime is not None:
            self.runtime.cancel()

    def restart_runtime_mode(self) -> None:
        if not self.runtime_mode_enabled:
            return

        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self._delete_runtime_snapshot()
        self._recreate_runtime()
        self.clear_runtime_log_view()
        self._enter_runtime_mode()

    def toggle_runtime_mode(self, checked: bool) -> None:
        if checked:
            self._enter_runtime_mode()
            return

        self.runtime_mode_enabled = False
        self.runtime_active_path = tuple()
        self.runtime_last_transition = None
        self._delete_runtime_snapshot()
        self._recreate_runtime()
        self.clear_runtime_log_view()
        self.render_current_container()
        self.set_canvas_runtime_visual_state()
        self.statusBar().showMessage("Runtime mode disabled", 3000)

    def set_canvas_runtime_visual_state(self) -> None:
        if hasattr(self, "runtime_controls_widget"):
            self.runtime_controls_widget.setVisible(self.runtime_mode_enabled)

        if hasattr(self, "editor_sidebar_widget"):
            self.editor_sidebar_widget.setVisible(not self.runtime_mode_enabled)
        if hasattr(self, "runtime_sidebar_widget"):
            self.runtime_sidebar_widget.setVisible(self.runtime_mode_enabled)

        if hasattr(self, "canvas_frame"):
            border_width = 3 if self.runtime_mode_enabled else 1
            border_color = (
                PALETTE.runtime_canvas_border
                if self.runtime_mode_enabled
                else PALETTE.ui_border
            )
            self.canvas_frame.setStyleSheet(
                f"QFrame {{ border: {border_width}px solid {border_color.name()}; }}"
            )

        if hasattr(self, "runtime_mode_button"):
            self.runtime_mode_button.setText(
                "Runtime Mode Active" if self.runtime_mode_enabled else "Runtime Mode"
            )
            if self.runtime_mode_enabled:
                self.runtime_mode_button.setStyleSheet(
                    "QPushButton {"
                    f"background-color: {PALETTE.runtime_mode_button_bg.name()}; "
                    f"color: {PALETTE.runtime_mode_button_text.name()}; "
                    f"border: 1px solid {PALETTE.runtime_canvas_border.name()};"
                    "}"
                )
            else:
                self.runtime_mode_button.setStyleSheet("")

    def _runtime_status_badge_style(self, status: str) -> str:
        """Return the stylesheet for the runtime status badge."""
        normalized = str(status).strip().lower()

        background = QColor(205, 210, 214)
        foreground = QColor(28, 31, 36)
        border = QColor(148, 154, 160)

        if normalized == "running":
            background = QColor(46, 150, 76)
            foreground = QColor(255, 255, 255)
            border = QColor(33, 108, 54)
        elif normalized == "paused":
            background = QColor(214, 170, 52)
            foreground = QColor(24, 24, 24)
            border = QColor(150, 118, 34)
        elif normalized == "inactive":
            background = QColor(145, 149, 156)
            foreground = QColor(255, 255, 255)
            border = QColor(103, 108, 115)
        elif normalized == "ready":
            background = QColor(224, 228, 232)
            foreground = QColor(30, 34, 40)
            border = QColor(172, 178, 184)
        elif normalized in {"succeeded", "success"}:
            background = QColor(46, 150, 76)
            foreground = QColor(255, 255, 255)
            border = QColor(33, 108, 54)
        elif normalized in {"aborted", "failed", "failure", "error"}:
            background = QColor(176, 60, 60)
            foreground = QColor(255, 255, 255)
            border = QColor(120, 40, 40)

        return (
            "QLabel {"
            f"background-color: {background.name()}; "
            f"color: {foreground.name()}; "
            f"border: 1px solid {border.name()}; "
            "border-radius: 10px; "
            "padding: 4px 12px; "
            "font-weight: 600;"
            "}"
        )

    def _update_runtime_status_badge(self, status: str) -> None:
        """Apply runtime status text and styling to the badge widget."""
        if not hasattr(self, "runtime_status_label"):
            return
        label_text = str(status).strip() or "Inactive"
        self.runtime_status_label.setText(label_text)
        self.runtime_status_label.setStyleSheet(
            self._runtime_status_badge_style(label_text)
        )

    def _runtime_log_uses_dark_background(self) -> bool:
        """Return whether the runtime log view uses a dark background."""
        return PALETTE.ui_input_bg.lightness() < 128

    def _runtime_log_view_style(self) -> str:
        """Return the stylesheet used by the runtime log view."""
        return (
            "QTextBrowser {"
            f"background-color: {PALETTE.ui_input_bg.name()}; "
            f"color: {PALETTE.text_primary.name()}; "
            f"border: 1px solid {PALETTE.ui_border.name()}; "
            f"selection-background-color: {PALETTE.ui_selection_bg.name()}; "
            f"selection-color: {PALETTE.ui_selection_text.name()}; "
            "border-radius: 6px;"
            "}"
        )

    def _runtime_log_line_color(self, message: str) -> str:
        """Return the HTML color used for a runtime log line."""
        normalized = str(message).lstrip()
        if self._runtime_log_uses_dark_background():
            if normalized.startswith("[WARN]"):
                return "#ffd24d"
            if normalized.startswith("[ERROR]"):
                return "#ff7a7a"
            if normalized.startswith("[STATUS]"):
                return "#d3d7dc"
            if normalized.startswith("[INFO]"):
                return "#ffffff"
            if normalized.startswith("[DEBUG]"):
                return "#9aa3ad"
            if normalized.startswith("[ACTIVE]"):
                return "#8dd8ff"
            if normalized.startswith("[TRANSITION]"):
                return "#78f0c8"
            return "#e8eaed"

        if normalized.startswith("[WARN]"):
            return "#8a6700"
        if normalized.startswith("[ERROR]"):
            return "#b53333"
        if normalized.startswith("[STATUS]"):
            return PALETTE.text_secondary.name()
        if normalized.startswith("[INFO]"):
            return PALETTE.text_primary.name()
        if normalized.startswith("[DEBUG]"):
            return "#6b7280"
        if normalized.startswith("[ACTIVE]"):
            return "#1565c0"
        if normalized.startswith("[TRANSITION]"):
            return "#0f8a6b"
        return PALETTE.text_primary.name()

    def _format_runtime_log_entry(self, message: str) -> str:
        """Convert a runtime log message into a styled HTML block."""
        color = self._runtime_log_line_color(message)
        return (
            '<div style="'
            f"color: {color}; "
            "font-family: 'DejaVu Sans Mono', 'Courier New', monospace; "
            "white-space: pre; "
            'margin: 0;">'
            f"{escape(str(message))}"
            "</div>"
        )

    def clear_runtime_log_view(self) -> None:
        if hasattr(self, "runtime_log_view"):
            self.runtime_log_view.clear()

    def append_runtime_log(self, message: str) -> None:
        if not hasattr(self, "runtime_log_view"):
            return
        self.runtime_log_view.append(self._format_runtime_log_entry(str(message)))
        scrollbar = self.runtime_log_view.verticalScrollBar()
        if scrollbar is not None:
            scrollbar.setValue(scrollbar.maximum())

    def on_runtime_active_state_changed(self, state_path: tuple[str, ...]) -> None:
        self.runtime_active_path = tuple(state_path or tuple())
        self._schedule_runtime_highlight_refresh()
        self.update_runtime_actions()

    def on_runtime_transition_changed(
        self,
        transition: Optional[tuple[tuple[str, ...], tuple[str, ...], str]],
    ) -> None:
        self.runtime_last_transition = transition
        self._schedule_runtime_highlight_refresh()

    def on_runtime_outcome_changed(self, outcome: Optional[str]) -> None:
        del outcome
        self._get_live_runtime_active_path()
        self.runtime_last_transition = None
        self._schedule_runtime_highlight_refresh()
        self.update_runtime_actions()

    def on_runtime_status_changed(self, message: str) -> None:
        self.statusBar().showMessage(message, 3000)
        self.append_runtime_log(f"[STATUS] {message}")
        runtime = self.runtime
        if runtime is not None:
            self._update_runtime_status_badge(runtime.get_status_label())
        self.update_runtime_actions()

    def on_runtime_error(self, message: str) -> None:
        self.statusBar().showMessage("Runtime error", 3000)
        self.append_runtime_log(f"[ERROR] {message}")
        self._update_runtime_status_badge("Error")
        QMessageBox.critical(self, "Runtime Error", message)
        self.update_runtime_actions()

    def update_runtime_actions(self) -> None:
        runtime = self.runtime
        runtime_ready = (
            self.runtime_mode_enabled and runtime is not None and runtime.is_ready()
        )
        runtime_running = runtime_ready and runtime.is_running()
        runtime_blocked = runtime_running and runtime.is_blocked()
        runtime_playing = runtime_running and not runtime_blocked
        runtime_step_mode = runtime_running and runtime.is_step_mode()
        runtime_finished = runtime_ready and runtime.is_finished()

        self._set_runtime_mode_button_checked(self.runtime_mode_enabled)
        self.set_canvas_runtime_visual_state()

        if hasattr(self, "runtime_status_label"):
            self._update_runtime_status_badge(
                runtime.get_status_label() if runtime is not None else "Inactive"
            )

        if hasattr(self, "runtime_log_view"):
            self.runtime_log_view.setStyleSheet(self._runtime_log_view_style())

        runtime_button_visibility = {
            "runtime_play_button": runtime_ready
            and not runtime_playing
            and not runtime_finished,
            "runtime_pause_button": runtime_playing and not runtime_step_mode,
            "runtime_step_button": runtime_ready
            and not runtime_playing
            and not runtime_finished,
            "runtime_cancel_state_button": runtime_running and not runtime_finished,
            "runtime_cancel_sm_button": runtime_running and not runtime_finished,
            "runtime_restart_button": runtime_ready,
        }
        for button_name, visible in runtime_button_visibility.items():
            button = getattr(self, button_name, None)
            if button is not None:
                button.setVisible(visible)
                button.setEnabled(visible)

        for action_name in [
            "new_action",
            "open_action",
        ]:
            action = getattr(self, action_name, None)
            if action is not None:
                action.setEnabled(not self.runtime_mode_enabled)

    def refresh_breadcrumbs(self) -> None:
        if not hasattr(self, "breadcrumb_layout"):
            return
        while self.breadcrumb_layout.count():
            item = self.breadcrumb_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        for index, container_model in enumerate(self.current_container_path):
            label = "root" if index == 0 else container_model.name
            button = QPushButton(label)
            button.setFlat(True)
            button.clicked.connect(
                lambda _checked=False, idx=index: self.navigate_to_container_index(idx)
            )
            self.breadcrumb_layout.addWidget(button)
            if index < len(self.current_container_path) - 1:
                self.breadcrumb_layout.addWidget(QLabel(">"))

        self.breadcrumb_layout.addStretch()

        fit_button = QPushButton("Fit")
        fit_button.clicked.connect(self.fit_current_view)
        self.breadcrumb_layout.addWidget(fit_button)

    def get_free_position(self) -> QPointF:
        """Get a free position close to the current viewport center."""
        if not hasattr(self, "canvas"):
            return QPointF(100, 100)

        viewport_rect = self.canvas.viewport().rect()
        visible_rect = self.canvas.mapToScene(viewport_rect).boundingRect()
        center = visible_rect.center()

        occupied_positions = [
            item.pos()
            for item in list(self.state_nodes.values())
            + list(self.final_outcomes.values())
        ]
        spacing_x = 180.0
        spacing_y = 130.0

        candidates = [QPointF(center.x(), center.y())]
        for radius in range(1, 6):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidates.append(
                        QPointF(
                            center.x() + dx * spacing_x, center.y() + dy * spacing_y
                        )
                    )

        def is_free(candidate: QPointF) -> bool:
            for occupied in occupied_positions:
                if (
                    abs(candidate.x() - occupied.x()) < spacing_x * 0.8
                    and abs(candidate.y() - occupied.y()) < spacing_y * 0.8
                ):
                    return False
            return True

        for candidate in candidates:
            if is_free(candidate):
                return candidate

        fallback_index = len(occupied_positions)
        return QPointF(
            center.x() + (fallback_index % 4) * spacing_x,
            center.y() + (fallback_index // 4) * spacing_y,
        )

    def load_from_xml(self, file_path: str) -> None:
        self.model_adapter.load_from_xml(file_path)
        self.current_file_path = file_path
        self.render_current_container(fit_view=True)

    def state_uses_blackboard_key(self, state_node, key_name: str) -> bool:
        def apply_remap_chain(raw_name: str, remap_chain: List[Dict[str, str]]) -> str:
            effective_name = raw_name
            for remappings in remap_chain:
                effective_name = remappings.get(effective_name, effective_name)
            return effective_name

        def model_uses_key(
            state_model: State, remap_chain: List[Dict[str, str]]
        ) -> bool:
            current_chain = remap_chain + [
                dict(getattr(state_model, "remappings", {}) or {})
            ]

            for key in getattr(state_model, "keys", []) or []:
                raw_name = str(getattr(key, "name", "") or "").strip()
                if raw_name and apply_remap_chain(raw_name, current_chain) == key_name:
                    return True

            if isinstance(state_model, (StateMachine, Concurrence)):
                for child_state in state_model.states.values():
                    if model_uses_key(child_state, current_chain):
                        return True
                return False

            try:
                plugin_info = self.resolve_plugin_info_for_model(state_model)
            except Exception:
                return False

            for key_info in list(getattr(plugin_info, "input_keys", []) or []) + list(
                getattr(plugin_info, "output_keys", []) or []
            ):
                plugin_key_name = str(key_info.get("name", "")).strip()
                if (
                    plugin_key_name
                    and apply_remap_chain(plugin_key_name, current_chain) == key_name
                ):
                    return True
            return False

        model = getattr(state_node, "model", None)
        if model is None:
            return False
        return model_uses_key(model, [])

    def update_blackboard_usage_highlighting(self) -> None:
        self.refresh_visual_highlighting()

    def _get_current_runtime_container_path(self) -> tuple[str, ...]:
        if self.current_runtime_container_path:
            return tuple(self.current_runtime_container_path)
        return tuple(
            str(container.name) for container in self.current_container_path[1:]
        )

    def _get_runtime_state_name_for_current_container(self) -> Optional[str]:
        if not self.runtime_mode_enabled:
            return None

        active_path = self._get_live_runtime_active_path()
        if not active_path:
            return None

        current_path = self._get_current_runtime_container_path()
        if len(current_path) >= len(active_path):
            return None

        if active_path[: len(current_path)] != current_path:
            return None

        return active_path[len(current_path)]

    def _find_runtime_connection_for_current_container(
        self,
    ) -> Optional[ConnectionLine]:
        transition = self._get_live_runtime_transition()
        if not self.runtime_mode_enabled or not transition:
            return None

        from_path, to_path, outcome = transition
        current_path = self._get_current_runtime_container_path()
        expected_depth = len(current_path) + 1

        if len(from_path) != expected_depth or len(to_path) != expected_depth:
            return None

        if from_path[: len(current_path)] != current_path:
            return None

        if to_path[: len(current_path)] != current_path:
            return None

        from_name = from_path[len(current_path)]
        to_name = to_path[len(current_path)]

        for connection in list(self.connections):
            if self._is_deleted_connection_item(connection):
                continue
            if (
                getattr(connection.from_node, "name", None) == from_name
                and getattr(connection.to_node, "name", None) == to_name
                and connection.outcome == outcome
            ):
                return connection

        return None

    def apply_runtime_highlighting(self) -> None:
        active_name = self._get_runtime_state_name_for_current_container()
        if not active_name:
            return

        active_item = self.state_nodes.get(active_name)
        if active_item is None or self._is_deleted_graphics_item(active_item):
            return

        try:
            active_item.setPen(QPen(PALETTE.runtime_highlight_pen, 5))
            active_item.setBrush(QBrush(PALETTE.runtime_highlight_fill))
        except RuntimeError:
            return

    def apply_runtime_transition_highlighting(self) -> None:
        connection = self._find_runtime_connection_for_current_container()
        if connection is None or self._is_deleted_connection_item(connection):
            return

        try:
            pen = QPen(PALETTE.runtime_highlight_pen, 5)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            connection.setPen(pen)
            connection.arrow_head.setBrush(QBrush(PALETTE.runtime_highlight_pen))
            connection.arrow_head.setPen(QPen(PALETTE.runtime_highlight_pen))
            connection.label_bg.setBrush(QBrush(PALETTE.runtime_highlight_fill))
            connection.label_bg.setPen(QPen(PALETTE.runtime_highlight_pen, 2))
        except RuntimeError:
            return

    def refresh_visual_highlighting(self) -> None:
        selected_key = self.get_selected_blackboard_key_name()

        stale_state_names = [
            state_name
            for state_name, state_node in self.state_nodes.items()
            if self._is_deleted_graphics_item(state_node)
        ]
        for state_name in stale_state_names:
            del self.state_nodes[state_name]

        stale_connections = [
            connection
            for connection in self.connections
            if self._is_deleted_connection_item(connection)
        ]
        for connection in stale_connections:
            self.connections.remove(connection)

        for connection in list(self.connections):
            self.apply_default_connection_visual_state(connection)

        for state_node in list(self.state_nodes.values()):
            self.apply_default_visual_state(state_node)

        if self._highlight_blackboard_usage and selected_key:
            for state_node in list(self.state_nodes.values()):
                if self._is_deleted_graphics_item(state_node):
                    continue
                if self.state_uses_blackboard_key(state_node, selected_key):
                    self.apply_default_visual_state(state_node)
                    try:
                        state_node.setPen(QPen(PALETTE.blackboard_highlight_pen, 5))
                        state_node.setBrush(QBrush(PALETTE.blackboard_highlight_fill))
                    except RuntimeError:
                        continue

        self.apply_runtime_highlighting()
        self.apply_runtime_transition_highlighting()

    def _ensure_external_xml_state(self) -> None:
        if not hasattr(self, "extern_xml"):
            self.extern_xml = None
        if not hasattr(self, "extern_xml_source_state"):
            self.extern_xml_source_state = None
        if not hasattr(self, "extern_xml_path_start_index"):
            self.extern_xml_path_start_index = None

    def is_read_only_mode(self) -> bool:
        self._ensure_external_xml_state()
        external_xml_read_only = (
            self.extern_xml is not None
            and self.extern_xml_path_start_index is not None
            and len(self.current_container_path) > self.extern_xml_path_start_index
        )
        return self.runtime_mode_enabled or external_xml_read_only

    def _show_read_only_message(self) -> None:
        if self.runtime_mode_enabled:
            self.statusBar().showMessage("Runtime mode is read-only", 3000)
        else:
            self.statusBar().showMessage("External XML view is read-only", 3000)

    def _clear_external_xml_view(self) -> None:
        self._ensure_external_xml_state()
        self.extern_xml = None
        self.extern_xml_source_state = None
        self.extern_xml_path_start_index = None

    def _resolve_xml_state_file_path(
        self, state_node: ContainerStateNode
    ) -> Optional[str]:
        state_model = getattr(state_node, "model", None)
        plugin_info = getattr(state_node, "plugin_info", None)
        if plugin_info is None and state_model is not None:
            try:
                plugin_info = self.resolve_plugin_info_for_model(state_model)
            except Exception:
                plugin_info = None

        candidates: List[str] = []
        for source in [plugin_info, state_model]:
            if source is None:
                continue
            for attr in [
                "file_path",
                "xml_file",
                "path",
                "full_path",
                "abs_path",
                "filepath",
                "file_name",
            ]:
                value = getattr(source, attr, None)
                if value:
                    candidates.append(str(value))

        for candidate in candidates:
            if os.path.isfile(candidate):
                return candidate

        package_name = None
        file_name = None
        for source in [plugin_info, state_model]:
            if source is None:
                continue
            if not package_name:
                package_name = getattr(source, "package_name", None)
            if not file_name:
                file_name = getattr(source, "file_name", None)

        if file_name and package_name:
            try:
                from ament_index_python.packages import \
                    get_package_share_directory

                share_dir = get_package_share_directory(str(package_name))
                direct_candidate = os.path.join(share_dir, str(file_name))
                if os.path.isfile(direct_candidate):
                    return direct_candidate
                for root_dir, _dirs, files in os.walk(share_dir):
                    if os.path.basename(str(file_name)) in files:
                        return os.path.join(root_dir, os.path.basename(str(file_name)))
            except Exception:
                pass

        return None

    def _state_has_available_outcomes(
        self,
        state_node: StateNode | ContainerStateNode,
    ) -> bool:
        if not hasattr(state_node, "model"):
            return False

        outcomes = list(getattr(state_node.model, "outcomes", []) or [])
        if not outcomes:
            return False

        if isinstance(self.current_container_model, Concurrence):
            return True

        used_outcomes = {
            transition.source_outcome
            for transition in self.current_container_model.transitions.get(
                state_node.name, []
            )
        }
        return any(outcome.name not in used_outcomes for outcome in outcomes)

    def refresh_connection_port_visibility(self) -> None:
        readonly = self.is_read_only_mode()
        for item in self.state_nodes.values():
            if hasattr(item, "connection_port"):
                item.connection_port.setVisible(
                    (not readonly) and self._state_has_available_outcomes(item)
                )

    def _set_scene_read_only_state(self) -> None:
        readonly = self.is_read_only_mode()
        for item in list(self.state_nodes.values()) + list(
            self.final_outcomes.values()
        ):
            item.setFlag(item.ItemIsMovable, not readonly)
            if hasattr(item, "connection_port") and readonly:
                item.connection_port.setVisible(False)
        if not readonly:
            self.refresh_connection_port_visibility()
        if hasattr(self, "root_sm_name_edit"):
            self.root_sm_name_edit.setEnabled(not readonly)
        if hasattr(self, "root_sm_description_edit"):
            self.root_sm_description_edit.setEnabled(not readonly)
        if hasattr(self, "start_state_combo"):
            self.start_state_combo.setEnabled(not readonly)
        for widget_name in [
            "python_list",
            "cpp_list",
            "xml_list",
            "python_filter",
            "cpp_filter",
            "xml_filter",
        ]:
            widget = getattr(self, widget_name, None)
            if widget is not None:
                widget.setEnabled(not readonly)

    def render_current_container(self, fit_view: bool = False) -> None:
        self.model_adapter.rebuild_scene(self.current_container_model)
        self.update_container_controls()
        self.update_start_state_combo()
        self.refresh_breadcrumbs()
        self.refresh_connection_port_visibility()
        self.refresh_blackboard_keys_list()
        self._set_scene_read_only_state()
        self.refresh_visual_highlighting()
        self.update_runtime_actions()
        if fit_view:
            self.fit_current_view()

    def navigate_to_container_index(self, index: int) -> None:
        self._ensure_external_xml_state()
        if index < 0 or index >= len(self.current_container_path):
            return
        if not self.is_read_only_mode():
            self.sync_current_container_layout()
        if (
            self.extern_xml_path_start_index is not None
            and index < self.extern_xml_path_start_index
        ):
            self.current_container_path = self.current_container_path[: index + 1]
            self.current_runtime_container_path = self.current_runtime_container_path[
                :index
            ]
            self._clear_external_xml_view()
        else:
            self.current_container_path = self.current_container_path[: index + 1]
            self.current_runtime_container_path = self.current_runtime_container_path[
                :index
            ]
        self.render_current_container(fit_view=True)

    def navigate_up_one_level(self) -> None:
        if len(self.current_container_path) > 1:
            self.navigate_to_container_index(len(self.current_container_path) - 2)

    def enter_container(self, container_node: ContainerStateNode) -> None:
        self._ensure_external_xml_state()
        if getattr(container_node, "is_xml_reference", False):
            external_xml_active = (
                self.extern_xml is not None
                and self.extern_xml_path_start_index is not None
                and len(self.current_container_path) > self.extern_xml_path_start_index
            )
            if external_xml_active:
                self.edit_state()
                return
            xml_file_path = self._resolve_xml_state_file_path(container_node)
            if not xml_file_path:
                QMessageBox.warning(
                    self,
                    "Error",
                    f"Unable to locate XML file for state '{container_node.name}'.",
                )
                return
            try:
                external_model = self.model_adapter.load_external_xml_model(
                    xml_file_path
                )
            except Exception as exc:
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to load external XML state machine:\n{exc}",
                )
                return
            self.sync_current_container_layout()
            self.extern_xml = external_model
            self.extern_xml_source_state = container_node.model
            self.extern_xml_path_start_index = len(self.current_container_path)
            self.current_container_path.append(external_model)
            self.current_runtime_container_path.append(container_node.name)
            self._get_live_runtime_active_path()
            self._get_live_runtime_transition()
            self.render_current_container(fit_view=True)
            self._schedule_runtime_highlight_refresh()
            self.statusBar().showMessage(
                f"Entered external XML: {container_node.name}", 2000
            )
            return

        self.sync_current_container_layout()
        self.current_container_path.append(container_node.model)
        self.current_runtime_container_path.append(container_node.name)
        self._get_live_runtime_active_path()
        self._get_live_runtime_transition()
        self.render_current_container(fit_view=True)
        self._schedule_runtime_highlight_refresh()
        self.statusBar().showMessage(f"Entered: {container_node.name}", 2000)

    def on_plugin_double_clicked(self, item: QListWidgetItem) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok:
            self.create_state_node(state_name, plugin_info, False, False)

    def on_xml_double_clicked(self, item: QListWidgetItem) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        xml_plugin = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name:"
        )
        if ok:
            self.create_state_node(state_name, xml_plugin, False, False)

    def add_state(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        all_plugins = (
            self.plugin_manager.python_plugins
            + self.plugin_manager.cpp_plugins
            + self.plugin_manager.xml_files
        )
        dialog = StatePropertiesDialog(available_plugins=all_plugins, parent=self)
        if dialog.exec_():
            result = dialog.get_state_data()
            if result[0]:
                name, plugin, outcomes, remappings, description, defaults = result
                self.create_state_node(
                    name,
                    plugin,
                    outcomes=outcomes,
                    remappings=remappings,
                    description=description,
                    defaults=defaults,
                )

    def add_state_machine(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        dialog = StateMachineDialog(parent=self)
        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:
                name, outcomes, start_state, remappings, description, defaults = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=True,
                    is_concurrence=False,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=start_state,
                    default_outcome=None,
                    description=description,
                    defaults=defaults,
                )

    def add_concurrence(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        dialog = ConcurrenceDialog(parent=self)
        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:
                name, outcomes, default_outcome, remappings, description, defaults = (
                    result
                )
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=False,
                    is_concurrence=True,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=None,
                    default_outcome=default_outcome,
                    description=description,
                    defaults=defaults,
                )

    def add_final_outcome(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        current_model = self.current_container_model
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        if ok and outcome_name:
            if outcome_name in self.final_outcomes:
                QMessageBox.warning(
                    self,
                    "Error",
                    f"Final outcome '{outcome_name}' already exists in this container!",
                )
                return

            x = 700
            y = len(self.final_outcomes) * 170
            model = Outcome(name=outcome_name)
            current_model.add_outcome(model)
            current_model.layout.set_outcome_position(outcome_name, x, y)
            self.model_adapter.create_final_outcome_view(model, x=x, y=y)

            if isinstance(current_model, Concurrence):
                if (
                    len(current_model.outcomes) == 1
                    and not current_model.default_outcome
                ):
                    current_model.default_outcome = outcome_name

            self.update_start_state_combo()
            self.refresh_connection_port_visibility()
            self.statusBar().showMessage(
                f"Added final outcome: {outcome_name}",
                2000,
            )

    def create_connection_from_drag(
        self, from_node: StateNode, to_node: StateNode
    ) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        current_model = self.current_container_model

        if isinstance(current_model, Concurrence):
            if not isinstance(to_node, FinalOutcomeNode) or isinstance(
                from_node, FinalOutcomeNode
            ):
                QMessageBox.warning(
                    self,
                    "Not Allowed",
                    "States inside a Concurrence can only connect to final outcomes of the current Concurrence.",
                )
                return
            outcomes_list = [outcome.name for outcome in from_node.model.outcomes]
        else:
            if isinstance(from_node, FinalOutcomeNode):
                QMessageBox.warning(
                    self,
                    "Not Allowed",
                    "Final outcomes of the current container cannot start transitions here.",
                )
                return
            outcomes_list = [outcome.name for outcome in from_node.model.outcomes]

        if not outcomes_list:
            QMessageBox.warning(
                self,
                "Error",
                "Cannot create transitions from states without outcomes!",
            )
            return

        if isinstance(current_model, StateMachine):
            used_outcomes = {
                transition.source_outcome
                for transition in current_model.transitions.get(from_node.name, [])
            }
            available_outcomes = [
                outcome_name
                for outcome_name in outcomes_list
                if outcome_name not in used_outcomes
            ]
            if not available_outcomes:
                QMessageBox.warning(
                    self, "Error", "All outcomes from this state are already used!"
                )
                return
        else:
            available_outcomes = outcomes_list

        if len(available_outcomes) == 1:
            outcome = available_outcomes[0]
            self.create_connection(from_node, to_node, outcome)
        else:
            outcome, ok = QInputDialog.getItem(
                self,
                "Select Outcome",
                f"Select outcome for transition from '{from_node.name}':",
                available_outcomes,
                0,
                False,
            )
            if ok:
                self.create_connection(from_node, to_node, outcome)

    def rewire_connection(
        self,
        connection: ConnectionLine,
        to_node: StateNode | ContainerStateNode | FinalOutcomeNode,
    ) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return

        if connection not in self.connections:
            return

        from_node = connection.from_node
        old_to_node = connection.to_node
        outcome = connection.outcome

        if old_to_node == to_node:
            self.statusBar().showMessage(
                f"Transition unchanged: {from_node.name} --[{outcome}]--> {to_node.name}",
                2000,
            )
            connection.setSelected(True)
            return

        self.unregister_connection_in_model(connection)
        connection.from_node.remove_connection(connection)
        connection.to_node.remove_connection(connection)
        self.canvas.scene.removeItem(connection)
        self.canvas.scene.removeItem(connection.arrow_head)
        self.canvas.scene.removeItem(connection.label_bg)
        self.canvas.scene.removeItem(connection.label)
        if connection in self.connections:
            self.connections.remove(connection)

        new_connection = self._create_connection_view(from_node, to_node, outcome)
        self.register_connection_in_model(from_node, to_node, outcome)
        self.refresh_connection_port_visibility()
        new_connection.setSelected(True)
        self.statusBar().showMessage(
            f"Rewired transition: {from_node.name} --[{outcome}]--> {to_node.name}",
            2000,
        )

    def create_connection(
        self, from_node: StateNode, to_node: StateNode, outcome: str
    ) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        current_model = self.current_container_model
        if isinstance(current_model, StateMachine):
            used_outcomes = {
                transition.source_outcome
                for transition in current_model.transitions.get(from_node.name, [])
            }
            if outcome in used_outcomes:
                QMessageBox.warning(
                    self,
                    "Error",
                    f"Outcome '{outcome}' is already used for a transition!",
                )
                return

        self._create_connection_view(from_node, to_node, outcome)
        self.register_connection_in_model(from_node, to_node, outcome)
        self.refresh_connection_port_visibility()
        self.statusBar().showMessage(
            f"Added transition: {from_node.name} --[{outcome}]--> {to_node.name}",
            2000,
        )

    def delete_selected(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        for item in self.canvas.scene.selectedItems():
            if isinstance(item, ConnectionLine):
                self.delete_connection_item(item)
                return
            if isinstance(item, FinalOutcomeNode):
                self.delete_final_outcome_item(item)
                return
            if isinstance(item, (StateNode, ContainerStateNode)):
                self.delete_state_item(item)
                return

    def edit_final_outcome(
        self, outcome_node: Optional[FinalOutcomeNode] = None
    ) -> None:
        if outcome_node is None:
            outcome_node = self.find_selected_item(FinalOutcomeNode)

        if outcome_node is None:
            QMessageBox.warning(self, "Error", "Please select a final outcome to edit!")
            return

        dialog = OutcomeDescriptionDialog(
            outcome_name=outcome_node.name,
            description=getattr(outcome_node, "description", ""),
            parent=self,
            readonly=self.is_read_only_mode(),
        )

        if self.is_read_only_mode():
            dialog.exec_()
            return

        if dialog.exec_():
            outcome_node.description = dialog.get_description()
            outcome_node.update_tooltip()
            self.statusBar().showMessage(
                f"Updated outcome description: {outcome_node.name}",
                2000,
            )

    def edit_selected_blackboard_key(
        self, item: Optional[QListWidgetItem] = None
    ) -> None:
        if item is None:
            item = self.blackboard_list.currentItem()
        if item is None:
            return

        key_data = dict(item.data(Qt.UserRole) or {})
        key_name = key_data.get("name", "")
        if not key_name:
            return

        metadata = dict(
            self._get_container_metadata_map(self.current_container_model).get(
                key_name, {}
            )
        )
        merged_key_data = {
            **key_data,
            **metadata,
            "name": key_data.get("name", ""),
            "key_type": key_data.get("key_type", "in"),
            "default_type": str(metadata.get("default_type", "") or ""),
            "default_value": str(metadata.get("default_value", "") or ""),
        }

        dlg = BlackboardKeyDialog(
            merged_key_data,
            parent=self,
            edit_mode=True,
            readonly=self.is_read_only_mode(),
        )
        if self.is_read_only_mode():
            dlg.exec_()
            return
        if dlg.exec_():
            updated_key = dlg.get_key_data()
            metadata_map = self._get_container_metadata_map(
                self.current_container_model
            )
            metadata_map[key_name] = {
                "description": updated_key.get("description", ""),
                "key_type": key_data.get("key_type", "in"),
                "default_type": updated_key.get("default_type", ""),
                "default_value": updated_key.get("default_value", ""),
            }
            self._set_container_metadata_map(self.current_container_model, metadata_map)
            self.sync_blackboard_keys()

    def _collect_container_key_lists(
        self, model: State
    ) -> tuple[List[Dict[str, str]], List[Dict[str, str]]]:
        input_keys: List[Dict[str, str]] = []
        output_keys: List[Dict[str, str]] = []
        for key in getattr(model, "keys", []) or []:
            key_data = {
                "name": str(getattr(key, "name", "") or ""),
                "description": str(getattr(key, "description", "") or ""),
                "default_type": str(getattr(key, "default_type", "") or ""),
                "default_value": (
                    ""
                    if getattr(key, "default_value", None) is None
                    else str(getattr(key, "default_value", ""))
                ),
                "has_default": bool(getattr(key, "default_type", "") or ""),
            }
            key_type = str(getattr(key, "key_type", "in") or "in")
            if key_type in ("in", "in/out"):
                input_keys.append(dict(key_data))
            if key_type in ("out", "in/out"):
                output_keys.append(dict(key_data))
        return input_keys, output_keys

    def edit_current_container(self) -> None:
        model = self.current_container_model
        input_keys, output_keys = self._collect_container_key_lists(model)
        dialog = StatePropertiesDialog(
            state_name=model.name,
            plugin_info=None,
            available_plugins=[],
            remappings=dict(model.remappings),
            outcomes=[outcome.name for outcome in model.outcomes],
            edit_mode=True,
            parent=self,
            description=getattr(model, "description", ""),
            defaults=[],
            fallback_input_keys=input_keys,
            fallback_output_keys=output_keys,
            container_kind=(
                "Concurrence" if isinstance(model, Concurrence) else "State Machine"
            ),
            readonly=self.is_read_only_mode(),
        )
        if self.is_read_only_mode():
            dialog.exec_()
            return

        if dialog.exec_():
            result = dialog.get_state_data()
            if result and result[0]:
                name, plugin, outcomes, remappings, description, defaults = result
                parent_model = self.current_parent_model
                old_name = model.name
                if parent_model is None:
                    model.name = name
                    if isinstance(model, StateMachine):
                        model.rename_transition_owner(old_name, name)
                else:
                    if name != old_name and name in parent_model.states:
                        QMessageBox.warning(
                            self, "Error", f"State '{name}' already exists!"
                        )
                        return
                    if name != old_name:
                        parent_model.rename_state(old_name, name)
                model.remappings.clear()
                model.remappings.update(remappings)
                self.update_container_controls()
                self.refresh_breadcrumbs()
                self.refresh_blackboard_keys_list()
                self.statusBar().showMessage(
                    f"Updated {'concurrence' if isinstance(model, Concurrence) else 'state machine'}: {name}",
                    2000,
                )

    def edit_state(self) -> None:
        state_node = self.find_selected_state_node()
        if not state_node:
            QMessageBox.warning(self, "Error", "Please select a state to edit!")
            return

        readonly = self.is_read_only_mode()
        is_container = isinstance(state_node, ContainerStateNode) and not getattr(
            state_node, "is_xml_reference", False
        )

        if is_container:
            input_keys, output_keys = self._collect_container_key_lists(
                state_node.model
            )
            dialog = StatePropertiesDialog(
                state_name=state_node.name,
                plugin_info=None,
                available_plugins=[],
                remappings=state_node.remappings,
                outcomes=[outcome.name for outcome in state_node.model.outcomes],
                edit_mode=True,
                parent=self,
                description=getattr(state_node, "description", ""),
                defaults=getattr(state_node, "defaults", []),
                fallback_input_keys=input_keys,
                fallback_output_keys=output_keys,
                container_kind=(
                    "Concurrence" if state_node.is_concurrence else "State Machine"
                ),
                readonly=readonly,
            )
            if readonly:
                dialog.exec_()
                return
            if dialog.exec_():
                result = dialog.get_state_data()
                if result and result[0]:
                    name, plugin, outcomes, remappings, description, defaults = result
                    if not self.apply_common_state_updates(
                        state_node,
                        name,
                        remappings,
                        getattr(state_node, "description", ""),
                        defaults,
                    ):
                        return
                    self.sync_blackboard_keys()
                    self.statusBar().showMessage(
                        f"Updated {'concurrence' if state_node.is_concurrence else 'state machine'}: {name}",
                        2000,
                    )
            return

        plugin_info = getattr(state_node, "plugin_info", None)
        if plugin_info is None and getattr(state_node, "model", None) is not None:
            try:
                plugin_info = self.resolve_plugin_info_for_model(state_node.model)
            except Exception:
                plugin_info = None

        dialog = StatePropertiesDialog(
            state_name=state_node.name,
            plugin_info=plugin_info,
            available_plugins=[plugin_info] if plugin_info else [],
            remappings=state_node.remappings,
            outcomes=[outcome.name for outcome in state_node.model.outcomes],
            edit_mode=True,
            parent=self,
            description=getattr(state_node, "description", ""),
            defaults=getattr(state_node, "defaults", []),
            readonly=readonly,
        )
        if readonly:
            dialog.exec_()
            return

        if dialog.exec_():
            result = dialog.get_state_data()
            if result[0]:
                name, plugin, outcomes, remappings, description, defaults = result
                if not self.apply_common_state_updates(
                    state_node,
                    name,
                    remappings,
                    description,
                    defaults,
                ):
                    return
                state_node.model.outcomes = [Outcome(name=item) for item in outcomes]
                self.sync_blackboard_keys()
                self.refresh_connection_port_visibility()
                self.statusBar().showMessage(f"Updated state: {name}", 2000)
