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
from typing import Dict, List, Optional, Set
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QListWidget,
    QLabel,
    QInputDialog,
    QMessageBox,
    QFileDialog,
    QSplitter,
    QListWidgetItem,
    QLineEdit,
    QComboBox,
    QAction,
    QToolBar,
    QDialog,
    QPushButton,
    QTextBrowser,
    QAbstractItemView,
)
from PyQt5.QtGui import QCloseEvent, QPen, QBrush, QColor
from PyQt5.QtCore import Qt, QPointF

from yasmin_plugins_manager.plugin_manager import PluginManager, PluginInfo
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas
from yasmin_editor.editor_gui.state_properties_dialog import StatePropertiesDialog
from yasmin_editor.editor_gui.state_machine_dialog import StateMachineDialog
from yasmin_editor.editor_gui.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.model_adapter import EditorModelAdapter
from yasmin_editor.editor_gui.blackboard_key_dialog import BlackboardKeyDialog
from yasmin_editor.editor_gui.outcome_description_dialog import OutcomeDescriptionDialog
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import validate_model


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

        self.model_adapter = EditorModelAdapter(self)
        self.create_ui()

        self.statusBar().showMessage("Loading plugins...")
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

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

    def get_root_state_nodes(self) -> Dict[str, StateNode | ContainerStateNode]:
        return {
            name: node
            for name, node in self.state_nodes.items()
            if getattr(node, "parent_container", None) is None
        }

    def get_container_path(self, container: Optional[ContainerStateNode]) -> str:
        names: List[str] = []
        current = container
        while current is not None:
            names.append(current.name)
            current = current.parent_container
        return ".".join(reversed(names))

    def get_state_node_key(
        self,
        name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        container_path = self.get_container_path(parent_container)
        return f"{container_path}.{name}" if container_path else name

    def register_state_node(
        self,
        state_node: StateNode | ContainerStateNode,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> str:
        key = self.get_state_node_key(state_node.name, parent_container)
        self.state_nodes[key] = state_node
        return key

    def find_selected_item(self, item_type):
        for item in self.canvas.scene.selectedItems():
            if isinstance(item, item_type):
                return item
        return None

    def find_selected_state_node(self) -> Optional[StateNode | ContainerStateNode]:
        return self.find_selected_item((StateNode, ContainerStateNode))

    def find_selected_container(self) -> Optional[ContainerStateNode]:
        return self.find_selected_item(ContainerStateNode)

    def get_selected_container_or_warn(self, message: str) -> Optional[ContainerStateNode]:
        container = self.find_selected_container()
        if container is None:
            QMessageBox.warning(self, "Error", message)
        return container

    def has_state_name_conflict(
        self,
        state_name: str,
        parent_container: Optional[ContainerStateNode] = None,
    ) -> bool:
        if parent_container is None:
            return state_name in self.get_root_state_nodes()
        return state_name in parent_container.child_states

    def add_model_state(
        self,
        model: State,
        defaults: Optional[List[Dict[str, str]]] = None,
        parent_container: Optional[ContainerStateNode] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> StateNode | ContainerStateNode:
        if parent_container is None:
            position = self.get_free_position()
            x = position.x() if x is None else x
            y = position.y() if y is None else y
            self.root_model.add_state(model)
            self.root_model.layout.set_state_position(model.name, x, y)
            node = self.model_adapter.create_state_view(model, x=x, y=y)
            self.update_start_state_combo()

            if len(self.get_root_state_nodes()) == 1 and not self.start_state:
                self.start_state = model.name
                index = self.start_state_combo.findText(model.name)
                if index >= 0:
                    self.start_state_combo.setCurrentIndex(index)
        else:
            node = self.model_adapter.create_state_view(
                model,
                parent_container=parent_container,
                x=0.0 if x is None else x,
                y=0.0 if y is None else y,
            )
            if parent_container.is_state_machine and len(parent_container.child_states) == 1:
                parent_container.start_state = model.name
                parent_container.update_start_state_label()

        node.defaults = defaults or []
        self.sync_blackboard_keys()
        return node

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

    def iter_state_subtree_items(self, state_node: StateNode | ContainerStateNode):
        yield state_node
        if isinstance(state_node, ContainerStateNode):
            for child_state in state_node.child_states.values():
                yield from self.iter_state_subtree_items(child_state)
            for final_outcome in state_node.final_outcomes.values():
                yield final_outcome

    def delete_state_item(self, state_node: StateNode | ContainerStateNode) -> None:
        connections_to_remove: List[ConnectionLine] = []
        for item in self.iter_state_subtree_items(state_node):
            for connection in getattr(item, "connections", []):
                if connection not in connections_to_remove:
                    connections_to_remove.append(connection)

        for connection in connections_to_remove:
            self.delete_connection_item(connection)

        parent_container = getattr(state_node, "parent_container", None)
        if parent_container is not None:
            parent_path = self.get_container_path(parent_container)
            self._remove_state_node_entries(state_node, parent_path)
            parent_container.remove_child_state(state_node.name)
            parent_container.auto_resize_for_children()
            self.canvas.scene.removeItem(state_node)
            self.sync_blackboard_keys()
            self.statusBar().showMessage(
                f"Deleted nested state: {state_node.name}",
                2000,
            )
            return

        self._remove_state_node_entries(state_node)
        self.root_model.remove_state(state_node.name)
        self.canvas.scene.removeItem(state_node)
        self.update_start_state_combo()
        self.sync_blackboard_keys()
        self.statusBar().showMessage(f"Deleted state: {state_node.name}", 2000)

    def delete_final_outcome_item(self, outcome_node: FinalOutcomeNode) -> None:
        for connection in outcome_node.connections[:]:
            self.delete_connection_item(connection)

        parent_container = outcome_node.parent_container
        if parent_container is not None:
            parent_container.final_outcomes.pop(outcome_node.name, None)
            parent_container.model.remove_outcome(outcome_node.name)
            parent_container.auto_resize_for_children()
            self.canvas.scene.removeItem(outcome_node)
        else:
            self.final_outcomes.pop(outcome_node.name, None)
            self.root_model.remove_outcome(outcome_node.name)
            self.canvas.scene.removeItem(outcome_node)

        self.statusBar().showMessage(
            f"Deleted final outcome: {outcome_node.name}",
            2000,
        )

    def closeEvent(self, event: QCloseEvent) -> None:
        """Handle window close event to ensure proper cleanup."""
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

    def create_ui(self) -> None:
        """Create and setup the user interface.

        Sets up the main window layout including toolbars, panels,
        and the state machine canvas.
        """
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        toolbar = QToolBar()
        self.addToolBar(toolbar)

        new_action = QAction("New", self)
        new_action.setShortcut("Ctrl+N")
        new_action.triggered.connect(self.new_state_machine)
        toolbar.addAction(new_action)

        open_action = QAction("Open", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self.open_state_machine)
        toolbar.addAction(open_action)

        save_action = QAction("Save", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_state_machine)
        toolbar.addAction(save_action)

        toolbar.addSeparator()

        add_state_action = QAction("Add State", self)
        add_state_action.triggered.connect(self.add_state)
        toolbar.addAction(add_state_action)

        add_state_machine_action = QAction("Add State Machine", self)
        add_state_machine_action.triggered.connect(self.add_state_machine)
        toolbar.addAction(add_state_machine_action)

        add_concurrence_action = QAction("Add Concurrence", self)
        add_concurrence_action.triggered.connect(self.add_concurrence)
        toolbar.addAction(add_concurrence_action)

        add_final_action = QAction("Add Final Outcome", self)
        add_final_action.triggered.connect(self.add_final_outcome)
        toolbar.addAction(add_final_action)

        toolbar.addSeparator()

        delete_action = QAction("Delete Selected", self)
        delete_action.triggered.connect(self.delete_selected)
        toolbar.addAction(delete_action)

        toolbar.addSeparator()

        help_action = QAction("Help", self)
        help_action.triggered.connect(self.show_help)
        toolbar.addAction(help_action)

        left_layout.addWidget(QLabel("<b>Blackboard Keys:</b>"))
        self.blackboard_filter = QLineEdit()
        self.blackboard_filter.setPlaceholderText("Filter blackboard keys...")
        self.blackboard_filter.textChanged.connect(self.filter_blackboard_keys)
        left_layout.addWidget(self.blackboard_filter)
        self.blackboard_list = QListWidget()
        self.blackboard_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.blackboard_list.itemSelectionChanged.connect(
            self.on_blackboard_selection_changed
        )
        self.blackboard_list.itemDoubleClicked.connect(self.edit_selected_blackboard_key)
        left_layout.addWidget(self.blackboard_list)
        blackboard_btn_row = QHBoxLayout()
        self.highlight_blackboard_btn = QPushButton("Highlight: On")
        self.highlight_blackboard_btn.setCheckable(True)
        self.highlight_blackboard_btn.setChecked(True)
        self.highlight_blackboard_btn.toggled.connect(self.toggle_blackboard_highlighting)
        blackboard_btn_row.addWidget(self.highlight_blackboard_btn)
        left_layout.addLayout(blackboard_btn_row)

        # Python states list
        left_layout.addWidget(QLabel("<b>Python States:</b>"))
        self.python_filter = QLineEdit()
        self.python_filter.setPlaceholderText("Filter Python states...")
        self.python_filter.textChanged.connect(
            lambda text: self.filter_list(self.python_list, text)
        )
        left_layout.addWidget(self.python_filter)
        self.python_list = QListWidget()
        self.python_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.python_list)

        # C++ states list
        left_layout.addWidget(QLabel("<b>C++ States:</b>"))
        self.cpp_filter = QLineEdit()
        self.cpp_filter.setPlaceholderText("Filter C++ states...")
        self.cpp_filter.textChanged.connect(
            lambda text: self.filter_list(self.cpp_list, text)
        )
        left_layout.addWidget(self.cpp_filter)
        self.cpp_list = QListWidget()
        self.cpp_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.cpp_list)

        left_layout.addWidget(QLabel("<b>XML State Machines:</b>"))
        self.xml_filter = QLineEdit()
        self.xml_filter.setPlaceholderText("Filter XML state machines...")
        self.xml_filter.textChanged.connect(
            lambda text: self.filter_list(self.xml_list, text)
        )
        left_layout.addWidget(self.xml_filter)
        self.xml_list = QListWidget()
        self.xml_list.itemDoubleClicked.connect(self.on_xml_double_clicked)
        left_layout.addWidget(self.xml_list)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        root_sm_widget = QWidget()
        root_sm_vlayout = QVBoxLayout(root_sm_widget)
        root_sm_vlayout.setContentsMargins(0, 0, 0, 0)

        root_sm_row1 = QHBoxLayout()
        root_sm_row1.addWidget(QLabel("<b>State Machine Name:</b>"))
        self.root_sm_name_edit = QLineEdit()
        self.root_sm_name_edit.setPlaceholderText("Enter root state machine name...")
        self.root_sm_name_edit.textChanged.connect(self.on_root_sm_name_changed)
        root_sm_row1.addWidget(self.root_sm_name_edit)

        root_sm_row1.addWidget(QLabel("<b>Start State:</b>"))
        self.start_state_combo = QComboBox()
        self.start_state_combo.addItem("(None)")
        self.start_state_combo.currentTextChanged.connect(self.on_start_state_changed)
        root_sm_row1.addWidget(self.start_state_combo)
        root_sm_vlayout.addLayout(root_sm_row1)

        root_sm_row2 = QHBoxLayout()
        root_sm_row2.addWidget(QLabel("<b>Description:</b>"))
        self.root_sm_description_edit = QLineEdit()
        self.root_sm_description_edit.setPlaceholderText("Enter FSM description...")
        self.root_sm_description_edit.textChanged.connect(self.on_root_sm_description_changed)
        root_sm_row2.addWidget(self.root_sm_description_edit)

        root_sm_vlayout.addLayout(root_sm_row2)

        right_layout.addWidget(root_sm_widget)

        canvas_header = QLabel(
            "<b>State Machine Canvas:</b> "
            "<i>(Drag from blue port to create transitions, scroll to zoom, right-click for options)</i>"
        )
        right_layout.addWidget(canvas_header)
        self.canvas = StateMachineCanvas()
        self.canvas.editor_ref = self
        right_layout.addWidget(self.canvas)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 1000])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.statusBar()

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

    def on_root_sm_name_changed(self, text: str) -> None:
        """Handle root state machine name change.

        Args:
            text: The new state machine name.
        """
        self.root_sm_name = text

    def on_root_sm_description_changed(self, text: str) -> None:
        """Handle root state machine description change."""
        self.root_model.description = text

    def on_start_state_changed(self, text: str) -> None:
        """Handle initial state selection change.

        Args:
            text: The selected state name or "(None)".
        """
        if text == "(None)":
            self.start_state = None
        else:
            self.start_state = text

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
                    "default_value": "" if key.default_value is None else str(key.default_value),
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

    def sync_blackboard_keys(self) -> None:
        derived_keys = self._collect_blackboard_key_usage()
        used_key_names = set(derived_keys.keys())
        self._blackboard_key_metadata = {
            key_name: metadata
            for key_name, metadata in self._blackboard_key_metadata.items()
            if key_name in used_key_names
        }
        self._blackboard_keys = list(derived_keys.values())
        self.root_model.keys = self.dicts_to_keys(self._blackboard_keys)
        self.refresh_blackboard_keys_list()

    def refresh_blackboard_keys_list(self) -> None:
        current_key_name = self.get_selected_blackboard_key_name()
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

    def get_selected_blackboard_key_name(self) -> Optional[str]:
        item = self.blackboard_list.currentItem()
        if item is None:
            return None
        key_data = item.data(Qt.UserRole) or {}
        return key_data.get("name")

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

        metadata = dict(self._blackboard_key_metadata.get(key_name, {}))
        merged_key_data = {
            **key_data,
            **metadata,
            "name": key_data.get("name", ""),
            "key_type": key_data.get("key_type", "in"),
            "default_type": str(metadata.get("default_type", "") or ""),
            "default_value": str(metadata.get("default_value", "") or ""),
        }

        dlg = BlackboardKeyDialog(merged_key_data, parent=self, edit_mode=True)
        if dlg.exec_():
            updated_key = dlg.get_key_data()
            self._blackboard_key_metadata[key_name] = {
                "description": updated_key.get("description", ""),
                "key_type": key_data.get("key_type", "in"),
                "default_type": updated_key.get("default_type", ""),
                "default_value": updated_key.get("default_value", ""),
            }
            self.sync_blackboard_keys()

    def set_blackboard_keys(self, keys: List[Dict[str, str]], sync: bool = True) -> None:
        self._blackboard_key_metadata = {}
        for key in keys:
            key_name = str(key.get("name", "") or "").strip()
            if not key_name:
                continue
            self._blackboard_key_metadata[key_name] = {
                "description": str(key.get("description", "") or "").strip(),
                "key_type": str(key.get("key_type", "in") or "in").strip(),
                "default_type": str(key.get("default_type", "") or "").strip(),
                "default_value": str(key.get("default_value", "") or "").strip(),
            }
        self.root_model.keys = self.dicts_to_keys(keys)
        self._blackboard_keys = [dict(item) for item in keys]
        if sync:
            self.sync_blackboard_keys()
        else:
            self.refresh_blackboard_keys_list()

    def get_blackboard_keys(self) -> List[Dict[str, str]]:
        self.sync_blackboard_keys()
        return self.keys_to_dicts(self.root_model.keys)

    def add_root_default_row(self) -> None:
        pass

    def remove_root_default_row(self) -> None:
        pass

    def add_root_default_row_with_data(self, data: dict) -> None:
        key_name = str(data.get("key", "") or "").strip()
        if not key_name:
            return
        self._blackboard_key_metadata[key_name] = {
            "description": str(data.get("description", "") or "").strip(),
            "key_type": "in",
            "default_type": str(data.get("type", "") or "").strip(),
            "default_value": str(data.get("value", "") or "").strip(),
        }
        self.sync_blackboard_keys()

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

    def _remove_state_node_entries(self, state_node: StateNode, prefix: str = "") -> None:
        full_name = f"{prefix}.{state_node.name}" if prefix else state_node.name
        if isinstance(state_node, ContainerStateNode):
            for child_state in list(state_node.child_states.values()):
                self._remove_state_node_entries(child_state, full_name)
        if full_name in self.state_nodes:
            del self.state_nodes[full_name]

    def state_uses_blackboard_key(self, state_node, key_name: str) -> bool:
        plugin_info = getattr(state_node, "plugin_info", None)
        if plugin_info is None:
            return False

        plugin_keys = []
        for key_info in list(getattr(plugin_info, "input_keys", []) or []) + list(
            getattr(plugin_info, "output_keys", []) or []
        ):
            plugin_key_name = str(key_info.get("name", "")).strip()
            if not plugin_key_name:
                continue
            plugin_keys.append(
                self.get_effective_blackboard_key_name(state_node, plugin_key_name)
            )

        return key_name in plugin_keys

    def apply_default_visual_state(self, item) -> None:
        is_selected = item.isSelected() if hasattr(item, "isSelected") else False

        if isinstance(item, StateNode):
            if item.plugin_info and item.plugin_info.plugin_type == "python":
                item.setBrush(QBrush(QColor(144, 238, 144)))
            elif item.plugin_info and item.plugin_info.plugin_type == "cpp":
                item.setBrush(QBrush(QColor(255, 182, 193)))
            else:
                item.setBrush(QBrush(QColor(255, 165, 0)))
            item.setPen(
                QPen(QColor(255, 200, 0), 4)
                if is_selected
                else QPen(QColor(0, 0, 180), 3)
            )
        elif isinstance(item, ContainerStateNode):
            if item.is_concurrence:
                item.setBrush(QBrush(QColor(255, 220, 150, 180)))
                item.setPen(QPen(QColor(255, 140, 0), 3))
            else:
                item.setBrush(QBrush(QColor(173, 216, 230, 180)))
                item.setPen(QPen(QColor(0, 0, 180), 3))
        elif isinstance(item, FinalOutcomeNode):
            item.setBrush(QBrush(QColor(255, 0, 0)))
            item.setPen(
                QPen(QColor(255, 200, 0), 4) if is_selected else QPen(QColor(0, 0, 0), 3)
            )

    def update_blackboard_usage_highlighting(self) -> None:
        selected_key = self.get_selected_blackboard_key_name()

        for state_node in self.state_nodes.values():
            self.apply_default_visual_state(state_node)

        if not self._highlight_blackboard_usage or not selected_key:
            return

        for state_node in self.state_nodes.values():
            if self.state_uses_blackboard_key(state_node, selected_key):
                state_node.setPen(QPen(QColor(255, 170, 0), 5))
                state_node.setBrush(QBrush(QColor(255, 255, 170)))

    def update_start_state_combo(self) -> None:
        """Update the initial state combo box with available states."""
        current = self.start_state
        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")

        for state_name in sorted(self.get_root_state_nodes().keys()):
            self.start_state_combo.addItem(state_name)

        if current:
            index = self.start_state_combo.findText(current)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)
            else:
                self.start_state = None
                self.start_state_combo.setCurrentIndex(0)
        else:
            self.start_state_combo.setCurrentIndex(0)

    def resolve_plugin_info_for_model(self, model: State) -> PluginInfo:
        if model.state_type == "py":
            plugin = next(
                (
                    item
                    for item in self.plugin_manager.python_plugins
                    if item.module == model.module and item.class_name == model.class_name
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
                    and (not model.package_name or item.package_name == model.package_name)
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
            raise ValueError(
                f"Unable to resolve plugin for state '{model.name}'"
            )
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
        for outcome_name in list(outcomes or getattr(plugin_info, "outcomes", []) or []):
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

    def reset_editor_state(self, model: Optional[StateMachine] = None) -> None:
        self.canvas.scene.clear()
        self.state_nodes.clear()
        self.final_outcomes.clear()
        self.connections.clear()
        self._blackboard_keys = []
        self._blackboard_key_metadata = {}
        self.root_sm_name_edit.blockSignals(True)
        self.root_sm_description_edit.blockSignals(True)
        self.root_sm_name_edit.clear()
        self.root_sm_description_edit.clear()
        self.root_sm_name_edit.blockSignals(False)
        self.root_sm_description_edit.blockSignals(False)
        self.root_model = model or self.model_adapter.create_empty_root_model()
        self.refresh_blackboard_keys_list()
        self.update_start_state_combo()

    def load_from_xml(self, file_path: str) -> None:
        self.model_adapter.load_from_xml(file_path)

    def save_to_xml(self, file_path: str) -> None:
        self.model_adapter.save_to_xml(file_path)

    def find_target_view(self, target_name: str, source_container=None):
        container = source_container
        while container is not None:
            if target_name in container.child_states:
                return container.child_states[target_name]
            if target_name in container.final_outcomes:
                return container.final_outcomes[target_name]
            container = container.parent_container

        root_state = self.get_root_state_nodes().get(target_name)
        if root_state is not None:
            return root_state
        return self.final_outcomes.get(target_name)

    def _create_connection_view(self, from_node, to_node, outcome: str) -> ConnectionLine:
        connection = ConnectionLine(from_node, to_node, outcome)
        self.canvas.scene.addItem(connection)
        self.canvas.scene.addItem(connection.arrow_head)
        self.canvas.scene.addItem(connection.label_bg)
        self.canvas.scene.addItem(connection.label)
        self.connections.append(connection)
        return connection

    def register_connection_in_model(self, from_node, to_node, outcome: str) -> None:
        source_container = getattr(from_node, "parent_container", None)
        target_name = to_node.name

        if (
            isinstance(from_node, FinalOutcomeNode)
            and source_container
            and source_container.parent_container
            and source_container.parent_container.is_concurrence
            and isinstance(to_node, FinalOutcomeNode)
            and to_node.parent_container == source_container.parent_container
        ):
            source_container.parent_container.model.set_outcome_rule(
                target_name,
                source_container.name,
                outcome,
            )
            return

        if isinstance(from_node, FinalOutcomeNode) and source_container and source_container.is_concurrence:
            owner_container = source_container.parent_container
            if owner_container is None:
                owner_model = self.root_model
            else:
                owner_model = owner_container.model
            if isinstance(owner_model, StateMachine):
                owner_model.add_transition(
                    source_container.name,
                    Transition(source_outcome=outcome, target=target_name),
                )
            return

        if source_container and isinstance(source_container.model, Concurrence):
            source_container.model.set_outcome_rule(target_name, from_node.name, outcome)
            return

        owner_model = self.root_model if source_container is None else source_container.model
        if isinstance(owner_model, StateMachine):
            owner_name = from_node.name
            owner_model.add_transition(
                owner_name,
                Transition(source_outcome=outcome, target=target_name),
            )

    def unregister_connection_in_model(self, connection: ConnectionLine) -> None:
        from_node = connection.from_node
        to_node = connection.to_node
        source_container = getattr(from_node, "parent_container", None)
        target_name = to_node.name

        if (
            isinstance(from_node, FinalOutcomeNode)
            and source_container
            and source_container.parent_container
            and source_container.parent_container.is_concurrence
            and isinstance(to_node, FinalOutcomeNode)
            and to_node.parent_container == source_container.parent_container
        ):
            source_container.parent_container.model.remove_outcome_rule(
                target_name,
                source_container.name,
            )
            return

        if isinstance(from_node, FinalOutcomeNode) and source_container and source_container.is_concurrence:
            owner_container = source_container.parent_container
            owner_model = self.root_model if owner_container is None else owner_container.model
            if isinstance(owner_model, StateMachine):
                owner_model.remove_transition(
                    source_container.name,
                    connection.outcome,
                    target_name,
                )
            return

        if source_container and isinstance(source_container.model, Concurrence):
            source_container.model.remove_outcome_rule(target_name, from_node.name)
            return

        owner_model = self.root_model if source_container is None else source_container.model
        if isinstance(owner_model, StateMachine):
            owner_model.remove_transition(
                from_node.name,
                connection.outcome,
                target_name,
            )

    def _rename_state_node_entries(self, old_prefix: str, new_prefix: str) -> None:
        updates = {}
        for key, value in list(self.state_nodes.items()):
            if key == old_prefix or key.startswith(old_prefix + "."):
                suffix = key[len(old_prefix):]
                updates[new_prefix + suffix] = value
                del self.state_nodes[key]
        self.state_nodes.update(updates)

    def _rename_state_node(self, state_node, new_name: str) -> None:
        old_name = state_node.name
        parent_container = getattr(state_node, "parent_container", None)
        parent_model = self.root_model if parent_container is None else parent_container.model
        old_prefix = self.get_state_node_key(old_name, parent_container)
        new_prefix = self.get_state_node_key(new_name, parent_container)

        parent_model.rename_state(old_name, new_name)

        if parent_container is not None:
            parent_container.child_states[new_name] = parent_container.child_states.pop(old_name)

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
                QMessageBox.warning(self, "Error", f"State '{new_name}' already exists!")
                return False
            self._rename_state_node(state_node, new_name)

        state_node.remappings = remappings
        state_node.description = description
        state_node.defaults = defaults
        return True

    def on_plugin_double_clicked(self, item: QListWidgetItem) -> None:
        """Handle double-click on a plugin item to add it as a state.

        Args:
            item: The list widget item that was double-clicked.
        """
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok:
            self.create_state_node(state_name, plugin_info, False, False)

    def on_xml_double_clicked(self, item: QListWidgetItem) -> None:
        """Handle double-click on an XML state machine to add it.

        Args:
            item: The list widget item that was double-clicked.
        """
        xml_plugin = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name:"
        )
        if ok:
            self.create_state_node(state_name, xml_plugin, False, False)

    def get_free_position(self) -> QPointF:
        """Get a free position in a deterministic grid layout.

        Uses a fixed grid starting from (100, 100) and places nodes in a predictable
        left-to-right, top-to-bottom pattern to ensure consistent positioning across
        different runs.

        Returns:
            QPointF: The calculated free position for a new node.
        """
        START_X = 100
        START_Y = 100
        NODE_WIDTH = 400
        NODE_HEIGHT = 350
        NODES_PER_ROW = 3

        root_nodes = list(self.get_root_state_nodes().values())

        all_items = list(root_nodes) + list(self.final_outcomes.values())

        num_items = len(all_items)
        row = num_items // NODES_PER_ROW
        col = num_items % NODES_PER_ROW

        x = START_X + (col * NODE_WIDTH)
        y = START_Y + (row * NODE_HEIGHT)

        return QPointF(x, y)

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

    def add_state(self) -> None:
        """Open dialog to add a new state to the state machine."""
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

    def add_state_machine(self) -> None:
        """Add a new State Machine container."""
        self.add_container(False)

    def add_concurrence(self) -> None:
        """Add a new Concurrence container."""
        self.add_container(True)

    def edit_state(self) -> None:
        """Edit properties of the selected state."""
        state_node = self.find_selected_state_node()

        if not state_node:
            QMessageBox.warning(self, "Error", "Please select a state to edit!")
            return

        old_name = state_node.name

        if isinstance(state_node, ContainerStateNode):
            if state_node.is_state_machine:
                child_state_names = list(state_node.child_states.keys())

                dialog = StateMachineDialog(
                    name=state_node.name,
                    outcomes=(
                        list(state_node.final_outcomes.keys())
                        if state_node.final_outcomes
                        else []
                    ),
                    start_state=state_node.start_state,
                    remappings=state_node.remappings,
                    child_states=child_state_names,
                    edit_mode=True,
                    parent=self,
                    description=getattr(state_node, "description", ""),
                    defaults=getattr(state_node, "defaults", []),
                )

                if dialog.exec_():
                    result = dialog.get_state_machine_data()
                    if result:
                        name, outcomes, start_state, remappings, description, defaults = (
                            result
                        )

                        if not self.apply_common_state_updates(
                            state_node,
                            name,
                            remappings,
                            description,
                            defaults,
                        ):
                            return

                        state_node.start_state = start_state
                        state_node.update_start_state_label()

                        self.sync_blackboard_keys()
                        self.statusBar().showMessage(
                            f"Updated state machine: {name}", 2000
                        )
            elif state_node.is_concurrence:
                final_outcome_names = (
                    list(state_node.final_outcomes.keys())
                    if state_node.final_outcomes
                    else []
                )

                dialog = ConcurrenceDialog(
                    name=state_node.name,
                    outcomes=(
                        list(state_node.final_outcomes.keys())
                        if state_node.final_outcomes
                        else []
                    ),
                    default_outcome=state_node.default_outcome,
                    remappings=state_node.remappings,
                    final_outcomes=final_outcome_names,
                    edit_mode=True,
                    parent=self,
                    description=getattr(state_node, "description", ""),
                    defaults=getattr(state_node, "defaults", []),
                )

                if dialog.exec_():
                    result = dialog.get_concurrence_data()
                    if result:
                        (
                            name,
                            outcomes,
                            default_outcome,
                            remappings,
                            description,
                            defaults,
                        ) = result

                        if not self.apply_common_state_updates(
                            state_node,
                            name,
                            remappings,
                            description,
                            defaults,
                        ):
                            return

                        state_node.default_outcome = default_outcome
                        state_node.update_default_outcome_label()

                        self.sync_blackboard_keys()
                        self.statusBar().showMessage(f"Updated concurrence: {name}", 2000)
        else:
            dialog = StatePropertiesDialog(
                state_name=state_node.name,
                plugin_info=(
                    state_node.plugin_info if hasattr(state_node, "plugin_info") else None
                ),
                available_plugins=(
                    [state_node.plugin_info]
                    if hasattr(state_node, "plugin_info") and state_node.plugin_info
                    else []
                ),
                remappings=state_node.remappings,
                outcomes=None,
                edit_mode=True,
                parent=self,
                description=getattr(state_node, "description", ""),
                defaults=getattr(state_node, "defaults", []),
            )

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
                    self.sync_blackboard_keys()
                    self.statusBar().showMessage(f"Updated state: {name}", 2000)

    def edit_final_outcome(self, outcome_node: Optional[FinalOutcomeNode] = None) -> None:
        """Edit the description of a final outcome."""
        if outcome_node is None:
            outcome_node = self.find_selected_item(FinalOutcomeNode)

        if outcome_node is None:
            QMessageBox.warning(self, "Error", "Please select a final outcome to edit!")
            return

        dialog = OutcomeDescriptionDialog(
            outcome_name=outcome_node.name,
            description=getattr(outcome_node, "description", ""),
            parent=self,
        )

        if dialog.exec_():
            outcome_node.description = dialog.get_description()
            outcome_node.update_tooltip()
            self.statusBar().showMessage(
                f"Updated outcome description: {outcome_node.name}",
                2000,
            )

    def add_state_to_container(self) -> None:
        """Add a child state to the selected container (SM or Concurrence)."""
        container = self.get_selected_container_or_warn(
            "Please select a user-created Container State Machine or Concurrence!"
        )
        if container is None:
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

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                model = self.create_leaf_model(
                    name=name,
                    plugin_info=plugin,
                    description=description,
                    remappings=remappings,
                    outcomes=outcomes,
                )
                self.add_model_state(model, defaults=defaults, parent_container=container)
                self.statusBar().showMessage(
                    f"Added state '{name}' to container '{container.name}'", 2000
                )

    def add_state_machine_to_container(self) -> None:
        """Add a State Machine to the selected container."""
        container = self.get_selected_container_or_warn(
            "Please select a user-created Container!"
        )
        if container is None:
            return

        dialog = StateMachineDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:
                name, outcomes, start_state, remappings, description, defaults = result

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                model = self.create_container_model(
                    name=name,
                    is_concurrence=False,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=start_state,
                    description=description,
                )
                self.add_model_state(model, defaults=defaults, parent_container=container)
                self.statusBar().showMessage(
                    f"Added State Machine '{name}' to container '{container.name}'", 2000
                )

    def add_concurrence_to_container(self) -> None:
        """Add a Concurrence to the selected container."""
        container = self.get_selected_container_or_warn(
            "Please select a user-created Container!"
        )
        if container is None:
            return

        dialog = ConcurrenceDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:
                name, outcomes, default_outcome, remappings, description, defaults = result

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                model = self.create_container_model(
                    name=name,
                    is_concurrence=True,
                    outcomes=outcomes,
                    remappings=remappings,
                    default_outcome=default_outcome,
                    description=description,
                )
                self.add_model_state(model, defaults=defaults, parent_container=container)
                self.statusBar().showMessage(
                    f"Added Concurrence '{name}' to container '{container.name}'", 2000
                )

    def create_connection_from_drag(
        self, from_node: StateNode, to_node: StateNode
    ) -> None:
        """Create a connection when user drags from one node to another.

        Args:
            from_node: The source node.
            to_node: The target node.
        """
        if isinstance(from_node, ContainerStateNode):
            QMessageBox.warning(
                self,
                "Not Allowed",
                "Container State Machines and Concurrence states cannot have external transitions.\n"
                "Use Final Outcomes inside the container to define exit points.",
            )
            return

        has_outcomes = False
        outcomes_list = []

        if hasattr(from_node, "plugin_info") and from_node.plugin_info:
            has_outcomes = True
            outcomes_list = list(from_node.plugin_info.outcomes)
        elif isinstance(from_node, FinalOutcomeNode) and from_node.inside_container:
            has_outcomes = True
            outcomes_list = [from_node.name]

        if not has_outcomes or not outcomes_list:
            QMessageBox.warning(
                self,
                "Error",
                "Cannot create transitions from states without outcomes!",
            )
            return

        is_in_concurrence = False
        parent_concurrence = None
        if hasattr(from_node, "parent_container") and from_node.parent_container:
            if isinstance(from_node.parent_container, ContainerStateNode):
                is_in_concurrence = from_node.parent_container.is_concurrence
                if is_in_concurrence:
                    parent_concurrence = from_node.parent_container

        if is_in_concurrence:
            is_valid_target = (
                isinstance(to_node, FinalOutcomeNode)
                and to_node.inside_container
                and to_node.parent_container == parent_concurrence
            ) or (
                isinstance(from_node, FinalOutcomeNode)
                and from_node.inside_container
                and to_node.parent_container != parent_concurrence
            )

            if not is_valid_target:
                QMessageBox.warning(
                    self,
                    "Not Allowed",
                    "States inside a Concurrence can only transition to Final Outcomes inside the same Concurrence.",
                )
                return

        if not is_in_concurrence:
            used_outcomes = (
                from_node.get_used_outcomes()
                if hasattr(from_node, "get_used_outcomes")
                else set()
            )
            available_outcomes = [
                o for o in outcomes_list if from_node.name + o not in used_outcomes
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

    def create_connection(
        self, from_node: StateNode, to_node: StateNode, outcome: str
    ) -> None:
        """Create and add a connection to the scene."""
        is_in_concurrence = False
        if hasattr(from_node, "parent_container") and from_node.parent_container:
            if isinstance(from_node.parent_container, ContainerStateNode):
                is_in_concurrence = from_node.parent_container.is_concurrence

        if not is_in_concurrence:
            if hasattr(from_node, "get_used_outcomes"):
                used_outcomes = from_node.get_used_outcomes()
                if outcome in used_outcomes:
                    QMessageBox.warning(
                        self,
                        "Error",
                        f"Outcome '{outcome}' is already used for a transition!",
                    )
                    return

        self._create_connection_view(from_node, to_node, outcome)
        self.register_connection_in_model(from_node, to_node, outcome)

        for conn in self.connections:
            if (conn.from_node == from_node and conn.to_node == to_node) or (
                conn.from_node == to_node and conn.to_node == from_node
            ):
                conn.update_position()

        self.statusBar().showMessage(
            f"Added transition: {from_node.name} --[{outcome}]--> {to_node.name}",
            2000,
        )

    def add_final_outcome(self) -> None:
        """Add a final outcome to the state machine or selected container."""
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        if ok and outcome_name:
            selected_container = self.find_selected_container()

            if selected_container:
                if outcome_name in selected_container.final_outcomes:
                    QMessageBox.warning(
                        self,
                        "Error",
                        f"Final outcome '{outcome_name}' already exists in this container!",
                    )
                    return

                rect = selected_container.rect()
                x = rect.right() - 100
                y = rect.top() + 70 + len(selected_container.final_outcomes) * 80

                model = Outcome(name=outcome_name)
                self.model_adapter.create_final_outcome_view(model, selected_container, x=x, y=y)

                if (
                    selected_container.is_concurrence
                    and len(selected_container.final_outcomes) == 1
                    and not selected_container.default_outcome
                ):
                    selected_container.default_outcome = outcome_name
                    selected_container.update_default_outcome_label()

                self.statusBar().showMessage(
                    f"Added final outcome '{outcome_name}' to container '{selected_container.name}'",
                    2000,
                )
            else:
                if outcome_name in self.final_outcomes:
                    QMessageBox.warning(
                        self, "Error", f"Final outcome '{outcome_name}' already exists!"
                    )
                    return

                x = 600
                y = len(self.final_outcomes) * 150
                model = Outcome(name=outcome_name)
                self.root_model.add_outcome(model)
                self.root_model.layout.set_outcome_position(outcome_name, x, y)
                self.model_adapter.create_final_outcome_view(model, x=x, y=y)
                self.statusBar().showMessage(f"Added final outcome: {outcome_name}", 2000)

    def delete_selected(self) -> None:
        """Delete the selected items from the canvas."""
        selected_items = self.canvas.scene.selectedItems()

        for item in selected_items:
            if isinstance(item, (StateNode, ContainerStateNode)):
                self.delete_state_item(item)
            elif isinstance(item, FinalOutcomeNode):
                self.delete_final_outcome_item(item)
            elif isinstance(item, ConnectionLine):
                self.delete_connection_item(item)
                self.statusBar().showMessage("Deleted transition", 2000)

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

    def save_state_machine(self) -> None:
        self.model_adapter.sync_root_model_from_ui()
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
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")
