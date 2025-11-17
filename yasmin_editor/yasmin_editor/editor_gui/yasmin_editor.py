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
import math
import random
from lxml import etree as ET
from typing import Dict, List
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
)
from PyQt5.QtCore import Qt, QPointF

from yasmin_editor.plugins_manager.plugin_manager import PluginManager
from yasmin_editor.plugins_manager.plugin_info import PluginInfo
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.state_machine_canvas import StateMachineCanvas
from yasmin_editor.editor_gui.state_properties_dialog import StatePropertiesDialog
from yasmin_editor.editor_gui.state_machine_dialog import StateMachineDialog
from yasmin_editor.editor_gui.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.transition_dialog import TransitionDialog


class YasminEditor(QMainWindow):

    def __init__(self, manager: PluginManager):
        super().__init__()
        self.setWindowTitle("YASMIN Editor")

        # Start maximized
        self.showMaximized()

        # Plugin manager
        self.plugin_manager = manager
        self.state_nodes: Dict[str, StateNode] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self.root_sm_name = ""  # Root state machine name
        self.start_state = None  # Initial state for root SM

        # Deterministic layout seed and RNG
        # By default, we set a stable seed so the graph layout is reproducible
        self.layout_seed = 42
        self.layout_rng = random.Random(self.layout_seed)

        # Create UI
        self.create_ui()

        # Load plugins in background
        self.statusBar().showMessage("Loading plugins...")
        QApplication.processEvents()
        self.populate_plugin_lists()
        self.statusBar().showMessage("Ready", 3000)

    def closeEvent(self, event):
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

    def create_ui(self):
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create splitter
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel - Available states
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Toolbar
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

        # Layout seed control - allows deterministic layouts
        set_seed_action = QAction("Set Layout Seed", self)
        set_seed_action.triggered.connect(self.set_layout_seed)
        toolbar.addAction(set_seed_action)
        apply_layout_action = QAction("Reapply Layout", self)
        apply_layout_action.triggered.connect(self.reapply_layout)
        toolbar.addAction(apply_layout_action)

        # Python states list
        left_layout.addWidget(QLabel("<b>Python States:</b>"))
        self.python_filter = QLineEdit()
        self.python_filter.setPlaceholderText("Filter Python states...")
        self.python_filter.textChanged.connect(self.filter_python_list)
        left_layout.addWidget(self.python_filter)
        self.python_list = QListWidget()
        self.python_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.python_list)

        # C++ states list
        left_layout.addWidget(QLabel("<b>C++ States:</b>"))
        self.cpp_filter = QLineEdit()
        self.cpp_filter.setPlaceholderText("Filter C++ states...")
        self.cpp_filter.textChanged.connect(self.filter_cpp_list)
        left_layout.addWidget(self.cpp_filter)
        self.cpp_list = QListWidget()
        self.cpp_list.itemDoubleClicked.connect(self.on_plugin_double_clicked)
        left_layout.addWidget(self.cpp_list)

        # XML state machines list
        left_layout.addWidget(QLabel("<b>XML State Machines:</b>"))
        self.xml_filter = QLineEdit()
        self.xml_filter.setPlaceholderText("Filter XML state machines...")
        self.xml_filter.textChanged.connect(self.filter_xml_list)
        left_layout.addWidget(self.xml_filter)
        self.xml_list = QListWidget()
        self.xml_list.itemDoubleClicked.connect(self.on_xml_double_clicked)
        left_layout.addWidget(self.xml_list)

        # Right panel - Canvas
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # Root SM configuration
        root_sm_widget = QWidget()
        root_sm_layout = QHBoxLayout(root_sm_widget)
        root_sm_layout.setContentsMargins(0, 0, 0, 0)

        root_sm_layout.addWidget(QLabel("<b>State Machine Name:</b>"))
        self.root_sm_name_edit = QLineEdit()
        self.root_sm_name_edit.setPlaceholderText("Enter root state machine name...")
        self.root_sm_name_edit.textChanged.connect(self.on_root_sm_name_changed)
        root_sm_layout.addWidget(self.root_sm_name_edit)

        root_sm_layout.addWidget(QLabel("<b>Start State:</b>"))
        self.start_state_combo = QComboBox()
        self.start_state_combo.addItem("(None)")
        self.start_state_combo.currentTextChanged.connect(self.on_start_state_changed)
        root_sm_layout.addWidget(self.start_state_combo)

        right_layout.addWidget(root_sm_widget)

        # Canvas header with instructions
        canvas_header = QLabel(
            "<b>State Machine Canvas:</b> "
            "<i>(Drag from blue port to create transitions, scroll to zoom, right-click for options)</i>"
        )
        right_layout.addWidget(canvas_header)
        self.canvas = StateMachineCanvas()
        self.canvas.editor_ref = self  # Set reference for drag-to-connect
        right_layout.addWidget(self.canvas)

        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        # Set initial sizes: left panel 300px, right panel takes remaining space
        splitter.setSizes([300, 1000])
        splitter.setStretchFactor(0, 0)  # Left panel doesn't stretch
        splitter.setStretchFactor(1, 1)  # Right panel stretches

        # Status bar
        self.statusBar()

    def set_layout_seed(self):
        """Open a dialog to set the layout seed. Set to -1 to use non-deterministic random."""
        seed, ok = QInputDialog.getInt(
            self,
            "Set Layout Seed",
            "Enter layout seed (-1 to disable deterministic layout):",
            value=(self.layout_seed if self.layout_seed is not None else -1),
        )

        if not ok:
            return

        if seed == -1:
            self.layout_seed = None
            self.layout_rng = None
            self.statusBar().showMessage("Layout seed disabled (non-deterministic)", 3000)
        else:
            self.layout_seed = int(seed)
            self.layout_rng = random.Random(self.layout_seed)
            self.statusBar().showMessage(f"Layout seed set to {seed}", 3000)

    def _reset_layout_rng(self):
        """Reset the layout RNG from the stored seed. If seed is None, use module-level randomness."""
        if getattr(self, "layout_seed", None) is None:
            self.layout_rng = None
        else:
            self.layout_rng = random.Random(self.layout_seed)

    def reapply_layout(self):
        """Re-seed RNG and re-run the layout over the entire document."""
        # Reset RNG and layouts
        self._reset_layout_rng()
        self._reorganize_all_containers()
        try:
            self._reposition_root_elements_after_resize()
        except Exception:
            pass

        # Update all connections positions
        for state in self.state_nodes.values():
            if hasattr(state, "connections"):
                for conn in state.connections:
                    conn.update_position()

        self.statusBar().showMessage("Reapplied layout", 2000)

    def populate_plugin_lists(self):
        # Populate Python list
        for plugin in self.plugin_manager.python_plugins:
            item = QListWidgetItem(f"{plugin.module}.{plugin.class_name}")
            item.setData(Qt.UserRole, plugin)
            self.python_list.addItem(item)

        # Populate C++ list
        for plugin in self.plugin_manager.cpp_plugins:
            item = QListWidgetItem(plugin.class_name)
            item.setData(Qt.UserRole, plugin)
            self.cpp_list.addItem(item)

        # Populate XML list
        for xml_plugin in self.plugin_manager.xml_files:
            display_name = f"{xml_plugin.package_name}/{xml_plugin.file_name}"
            item = QListWidgetItem(display_name)
            item.setData(Qt.UserRole, xml_plugin)
            self.xml_list.addItem(item)

    def filter_python_list(self, text):
        """Filter Python states list based on search text."""
        for i in range(self.python_list.count()):
            item = self.python_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_cpp_list(self, text):
        """Filter C++ states list based on search text."""
        for i in range(self.cpp_list.count()):
            item = self.cpp_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_xml_list(self, text):
        """Filter XML state machines list based on search text."""
        for i in range(self.xml_list.count()):
            item = self.xml_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def on_root_sm_name_changed(self, text):
        """Handle root state machine name change."""
        self.root_sm_name = text

    def on_start_state_changed(self, text):
        """Handle initial state selection change."""
        if text == "(None)":
            self.start_state = None
        else:
            self.start_state = text

    def update_start_state_combo(self):
        """Update the initial state combo box with available states."""
        current = self.start_state
        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")

        for state_name in self.state_nodes.keys():
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

    def on_plugin_double_clicked(self, item: QListWidgetItem):
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok:
            self.create_state_node(state_name, plugin_info, False, False)

    def on_xml_double_clicked(self, item: QListWidgetItem):
        xml_plugin = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name:"
        )
        if ok:
            # For XML state machines, pass the plugin_info
            self.create_state_node(state_name, xml_plugin, False, False)

    def get_free_position(self):
        """Get a free position in a deterministic grid layout.

        This method:
        1. Uses a fixed grid starting from (100, 100)
        2. Places nodes in a predictable left-to-right, top-to-bottom pattern
        3. Ensures consistent positioning across different runs
        """
        # Grid parameters for deterministic layout
        START_X = 100
        START_Y = 100
        NODE_WIDTH = 400  # Width including spacing
        NODE_HEIGHT = 350  # Height including spacing
        NODES_PER_ROW = 3  # Number of nodes per row

        # Get root-level nodes only
        root_nodes = [
            node
            for node in self.state_nodes.values()
            if not hasattr(node, "parent_container") or node.parent_container is None
        ]

        # Add final outcomes to the list of items to count
        all_items = list(root_nodes) + list(self.final_outcomes.values())

        # Calculate position based on number of existing items
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
    ):
        if not name:
            QMessageBox.warning(self, "Validation Error", "Name is required!")
            return

        if name in self.state_nodes:
            QMessageBox.warning(self, "Error", f"State '{name}' already exists!")
            return

        pos = self.get_free_position()

        if is_state_machine or is_concurrence:
            node = ContainerStateNode(
                name,
                pos.x(),
                pos.y(),
                is_concurrence,
                remappings,
                outcomes,
                start_state,
                default_outcome,
            )
        else:
            node = StateNode(name, plugin_info, pos.x(), pos.y(), remappings)

        self.canvas.scene.addItem(node)
        self.state_nodes[name] = node
        self.update_start_state_combo()

        if len(self.state_nodes) == 1 and not self.start_state:
            self.start_state = name
            index = self.start_state_combo.findText(name)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)

        self.statusBar().showMessage(f"Added state: {name}", 2000)

    def add_state(self):
        all_plugins = (
            self.plugin_manager.python_plugins
            + self.plugin_manager.cpp_plugins
            + self.plugin_manager.xml_files
        )
        dialog = StatePropertiesDialog(
            available_plugins=all_plugins,
            parent=self,
        )

        if dialog.exec_():
            result = dialog.get_state_data()
            if result[0]:  # Check if name is not None (validation passed)
                (
                    name,
                    plugin,
                    outcomes,
                    remappings,
                ) = result
                self.create_state_node(
                    name,
                    plugin,
                    is_state_machine=False,
                    is_concurrence=False,
                    outcomes=outcomes,
                    remappings=remappings,
                )

    def add_state_machine(self):
        """Add a new State Machine container."""
        dialog = StateMachineDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:  # Check if validation passed
                name, outcomes, start_state, remappings = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=True,
                    is_concurrence=False,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=start_state,
                    default_outcome=None,
                )

    def add_concurrence(self):
        """Add a new Concurrence container."""
        dialog = ConcurrenceDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:  # Check if validation passed
                name, outcomes, default_outcome, remappings = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=False,
                    is_concurrence=True,
                    outcomes=outcomes,
                    remappings=remappings,
                    start_state=None,
                    default_outcome=default_outcome,
                )

    def edit_state(self):
        """Edit properties of the selected state."""
        selected_items = self.canvas.scene.selectedItems()
        state_node = None

        for item in selected_items:
            if isinstance(item, (StateNode, ContainerStateNode)):
                state_node = item
                break

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
                )

                if dialog.exec_():
                    result = dialog.get_state_machine_data()
                    if result:
                        name, outcomes, start_state, remappings = result

                        if name != old_name:
                            if name in self.state_nodes:
                                QMessageBox.warning(
                                    self, "Error", f"State '{name}' already exists!"
                                )
                                return

                            if self.start_state == old_name:
                                self.start_state = name

                            del self.state_nodes[old_name]
                            self.state_nodes[name] = state_node
                            state_node.name = name
                            state_node.title.setPlainText(f"STATE MACHINE: {name}")
                            title_rect = state_node.title.boundingRect()
                            state_node.title.setPos(-title_rect.width() / 2, -75)
                            self.update_start_state_combo()

                        state_node.remappings = remappings

                        if start_state:
                            state_node.start_state = start_state
                            state_node.update_start_state_label()

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
                )

                if dialog.exec_():
                    result = dialog.get_concurrence_data()
                    if result:
                        name, outcomes, default_outcome, remappings = result

                        if name != old_name:
                            if name in self.state_nodes:
                                QMessageBox.warning(
                                    self, "Error", f"State '{name}' already exists!"
                                )
                                return

                            if self.start_state == old_name:
                                self.start_state = name

                            del self.state_nodes[old_name]
                            self.state_nodes[name] = state_node
                            state_node.name = name
                            state_node.title.setPlainText(f"CONCURRENCE: {name}")
                            title_rect = state_node.title.boundingRect()
                            state_node.title.setPos(-title_rect.width() / 2, -75)
                            self.update_start_state_combo()

                        state_node.remappings = remappings

                        if default_outcome:
                            state_node.default_outcome = default_outcome
                            state_node.update_default_outcome_label()

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
            )

            if dialog.exec_():
                result = dialog.get_state_data()
                if result[0]:
                    name, plugin, outcomes, remappings = result

                    if name != old_name:
                        if name in self.state_nodes:
                            QMessageBox.warning(
                                self, "Error", f"State '{name}' already exists!"
                            )
                            return

                        if self.start_state == old_name:
                            self.start_state = name

                        del self.state_nodes[old_name]
                        self.state_nodes[name] = state_node
                        state_node.name = name
                        state_node.text.setPlainText(name)
                        text_rect = state_node.text.boundingRect()
                        state_node.text.setPos(
                            -text_rect.width() / 2, -text_rect.height() / 2
                        )
                        self.update_start_state_combo()

                    state_node.remappings = remappings
                    self.statusBar().showMessage(f"Updated state: {name}", 2000)

    def add_state_to_container(self):
        """Add a child state to the selected container (SM or Concurrence)."""
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                container = item
                break

        if not container:
            QMessageBox.warning(
                self,
                "Error",
                "Please select a user-created Container State Machine or Concurrence!",
            )
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
                name, plugin, outcomes, remappings = result

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                child_node = StateNode(name, plugin, 0, 0, remappings)
                container.add_child_state(child_node)
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_node

                self.statusBar().showMessage(
                    f"Added state '{name}' to container '{container.name}'", 2000
                )

                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def add_state_machine_to_container(self):
        """Add a State Machine to the selected container."""
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                container = item
                break

        if not container:
            QMessageBox.warning(self, "Error", "Please select a user-created Container!")
            return

        dialog = StateMachineDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:
                name, outcomes, start_state, remappings = result

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                child_sm = ContainerStateNode(
                    name, 0, 0, False, remappings, outcomes, start_state, None
                )
                container.add_child_state(child_sm)
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_sm

                self.statusBar().showMessage(
                    f"Added State Machine '{name}' to container '{container.name}'", 2000
                )

                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def add_concurrence_to_container(self):
        """Add a Concurrence to the selected container."""
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                container = item
                break

        if not container:
            QMessageBox.warning(self, "Error", "Please select a user-created Container!")
            return

        dialog = ConcurrenceDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:
                name, outcomes, default_outcome, remappings = result

                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                child_cc = ContainerStateNode(
                    name, 0, 0, True, remappings, outcomes, None, default_outcome
                )
                container.add_child_state(child_cc)
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_cc

                self.statusBar().showMessage(
                    f"Added Concurrence '{name}' to container '{container.name}'", 2000
                )

                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def create_connection_from_drag(self, from_node, to_node):
        """Create a connection when user drags from one node to another."""
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

    def create_connection(self, from_node, to_node, outcome: str):
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

        connection = ConnectionLine(from_node, to_node, outcome)
        self.canvas.scene.addItem(connection)
        self.canvas.scene.addItem(connection.arrow_head)
        self.canvas.scene.addItem(connection.label_bg)
        self.canvas.scene.addItem(connection.label)
        self.connections.append(connection)

        for conn in self.connections:
            if (conn.from_node == from_node and conn.to_node == to_node) or (
                conn.from_node == to_node and conn.to_node == from_node
            ):
                conn.update_position()

        self.statusBar().showMessage(
            f"Added transition: {from_node.name} --[{outcome}]--> {to_node.name}",
            2000,
        )

    def add_transition(self):
        selected_items = self.canvas.scene.selectedItems()
        from_state = None

        for item in selected_items:
            if isinstance(item, StateNode):
                from_state = item
                break

        if not from_state:
            QMessageBox.warning(
                self, "Error", "Please select a regular state (not a container) first!"
            )
            return

        has_outcomes = False
        if hasattr(from_state, "plugin_info") and from_state.plugin_info:
            has_outcomes = True

        if not has_outcomes:
            QMessageBox.warning(self, "Error", "Selected state has no outcomes!")
            return

        is_in_concurrence = False
        parent_concurrence = None
        if hasattr(from_state, "parent_container") and from_state.parent_container:
            if isinstance(from_state.parent_container, ContainerStateNode):
                is_in_concurrence = from_state.parent_container.is_concurrence
                if is_in_concurrence:
                    parent_concurrence = from_state.parent_container

        available_targets = []

        if is_in_concurrence:
            available_targets = [
                outcome for outcome in parent_concurrence.final_outcomes.values()
            ]
            if not available_targets:
                QMessageBox.warning(
                    self,
                    "Error",
                    "No final outcomes available in the Concurrence!\n"
                    "States inside a Concurrence can only transition to Final Outcomes inside the same Concurrence.",
                )
                return
        else:
            for state in self.state_nodes.values():
                if not isinstance(state, ContainerStateNode):
                    available_targets.append(state)
            available_targets.extend(list(self.final_outcomes.values()))
            available_targets = [t for t in available_targets if t != from_state]

            if not available_targets:
                QMessageBox.warning(self, "Error", "No target states available!")
                return

        dialog = TransitionDialog(from_state, available_targets, self)

        if dialog.exec_():
            outcome, target = dialog.get_transition_data()
            if outcome:
                self.create_connection(from_state, target, outcome)

    def add_final_outcome(self):
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        if ok and outcome_name:
            selected_items = self.canvas.scene.selectedItems()
            selected_container = None

            for item in selected_items:
                if isinstance(item, ContainerStateNode):
                    selected_container = item
                    break

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

                node = FinalOutcomeNode(outcome_name, x, y, inside_container=True)
                node.parent_container = selected_container
                node.setParentItem(selected_container)
                selected_container.final_outcomes[outcome_name] = node
                selected_container.auto_resize_for_children()

                if (
                    selected_container.is_concurrence
                    and len(selected_container.final_outcomes) == 1
                ):
                    if not selected_container.default_outcome:
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
                node = FinalOutcomeNode(outcome_name, x, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome_name] = node
                self.statusBar().showMessage(f"Added final outcome: {outcome_name}", 2000)

    def delete_selected(self):
        selected_items = self.canvas.scene.selectedItems()

        for item in selected_items:
            if isinstance(item, (StateNode, ContainerStateNode)):
                for connection in item.connections[:]:
                    if connection.from_node == item:
                        connection.to_node.remove_connection(connection)
                    else:
                        connection.from_node.remove_connection(connection)

                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                if hasattr(item, "parent_container") and item.parent_container:
                    parent = item.parent_container

                    if item.name in parent.child_states:
                        del parent.child_states[item.name]

                    full_name = f"{parent.name}.{item.name}"
                    if full_name in self.state_nodes:
                        del self.state_nodes[full_name]

                    parent.auto_resize_for_children()
                    self.canvas.scene.removeItem(item)
                    self.statusBar().showMessage(
                        f"Deleted nested state: {item.name}", 2000
                    )
                else:
                    self.canvas.scene.removeItem(item)
                    if item.name in self.state_nodes:
                        del self.state_nodes[item.name]
                    self.update_start_state_combo()
                    self.statusBar().showMessage(f"Deleted state: {item.name}", 2000)

            elif isinstance(item, FinalOutcomeNode):
                for connection in item.connections[:]:
                    if connection.from_node == item:
                        connection.to_node.remove_connection(connection)
                    else:
                        connection.from_node.remove_connection(connection)

                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                if item.parent_container:
                    if item.name in item.parent_container.final_outcomes:
                        del item.parent_container.final_outcomes[item.name]
                    item.parent_container.auto_resize_for_children()
                    self.canvas.scene.removeItem(item)
                else:
                    self.canvas.scene.removeItem(item)
                    if item.name in self.final_outcomes:
                        del self.final_outcomes[item.name]

                self.statusBar().showMessage(f"Deleted final outcome: {item.name}", 2000)

            elif isinstance(item, ConnectionLine):
                item.from_node.remove_connection(item)
                item.to_node.remove_connection(item)
                self.canvas.scene.removeItem(item)
                self.canvas.scene.removeItem(item.arrow_head)
                self.canvas.scene.removeItem(item.label_bg)
                self.canvas.scene.removeItem(item.label)
                if item in self.connections:
                    self.connections.remove(item)
                self.statusBar().showMessage("Deleted transition", 2000)

    def show_help(self):
        """Display help dialog with usage instructions."""
        help_text = """
        <h2>YASMIN Editor - Quick Guide</h2>
        
        <h3>File Operations</h3>
        <b>New/Open/Save:</b> Create, load, or save state machines from XML files.
        
        <h3>Building State Machines</h3>
        <b>State Machine Name:</b> Set the root state machine name.<br>
        <b>Start State:</b> Select the first state to execute.<br>
        <b>Add State:</b> Add a regular state (Python/C++/XML-based).<br>
        <b>Add State Machine:</b> Add a nested state machine container.<br>
        <b>Add Concurrence:</b> Add a concurrent execution container.<br>
        <b>Add Final Outcome:</b> Add an exit point for the state machine.
        
        <h3>Working with States</h3>
        <b>Double-click:</b> A plugin in the left panel to quickly add a state.<br>
        <b>Right-click:</b> On a state for options (edit, delete, add transitions).<br>
        <b>Drag:</b> States to reposition them on the canvas.<br>
        <b>Delete Selected:</b> Select items and click "Delete Selected" button.
        
        <h3>Creating Transitions</h3>
        <b>Drag from blue port:</b> Click and drag from the blue connection port to another state.<br>
        <b>Select outcome:</b> Choose which outcome triggers the transition.<br>
        
        <h3>Containers</h3>
        <b>Nested States:</b> Double-click a container to view/edit internal states.<br>
        <b>Final Outcomes:</b> Containers use final outcomes as exit points.<br>
        <b>State Machine:</b> Sequential execution based on start state.<br>
        <b>Concurrence:</b> All child states execute in parallel.
        
        <h3>Canvas Navigation</h3>
        <b>Scroll:</b> Zoom in/out.<br>
        <b>Drag:</b> Move states or pan the view.
        
        <h3>Validation (before saving)</h3>
        • State machine name is set<br>
        • Start state is selected<br>
        • At least one final outcome exists<br>
        • All states have unique names
        
        <h3>Tips</h3>
        • Use filters in left panel to find states quickly.<br>
        • Container states auto-resize to fit children.<br>
        • States in Concurrence can only transition to final outcomes within that Concurrence.<br>
        • XML state machines appear as regular states (not containers).
        """

        # Create custom dialog with scrollable content
        dialog = QDialog(self)
        dialog.setWindowTitle("YASMIN Editor Help")
        dialog.setMinimumSize(600, 500)
        dialog.setMaximumSize(800, 600)

        layout = QVBoxLayout(dialog)

        # Text browser for scrollable content
        text_browser = QTextBrowser()
        text_browser.setHtml(help_text)
        text_browser.setOpenExternalLinks(False)
        layout.addWidget(text_browser)

        # OK button
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(dialog.accept)
        ok_button.setDefault(True)
        button_layout.addWidget(ok_button)
        layout.addLayout(button_layout)

        dialog.exec_()

    def new_state_machine(self):
        reply = QMessageBox.question(
            self,
            "New State Machine",
            "Are you sure you want to create a new state machine? All unsaved changes will be lost.",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self.canvas.scene.clear()
            self.state_nodes.clear()
            self.final_outcomes.clear()
            self.connections.clear()
            self.root_sm_name = ""
            self.start_state = None
            self.root_sm_name_edit.clear()
            self.update_start_state_combo()
            self.statusBar().showMessage("New state machine created", 2000)
            return True

        return False

    def open_state_machine(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open State Machine", "", "XML Files (*.xml)"
        )

        if file_path:
            try:
                self.load_from_xml(file_path)
                self.statusBar().showMessage(f"Opened: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to open file: {str(e)}")

    def save_state_machine(self):
        # Validation checks
        errors = []

        if not self.final_outcomes:
            errors.append("- No final outcomes defined")

        if not self.root_sm_name or not self.root_sm_name.strip():
            errors.append("- Root state machine name is empty")

        if not self.state_nodes:
            errors.append("- No states defined")

        if not self.start_state:
            errors.append("- Initial state is not set")
        elif self.start_state not in self.state_nodes:
            errors.append(f"- Initial state '{self.start_state}' does not exist")

        # Check for states with empty names
        for name in self.state_nodes.keys():
            if not name or not name.strip():
                errors.append("- Found state with empty name")
                break

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
            # Ensure .xml extension
            if not file_path.lower().endswith(".xml"):
                file_path += ".xml"

            try:
                self.save_to_xml(file_path)
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def save_to_xml(self, file_path: str):
        # Use root SM name if set, otherwise ask
        sm_name = self.root_sm_name
        if not sm_name:
            sm_name, ok = QInputDialog.getText(
                self, "State Machine Name", "Enter state machine name (optional):"
            )
        else:
            ok = True

        # Create root element
        root = ET.Element("StateMachine")
        root.set("name", sm_name)
        root.set("outcomes", " ".join(self.final_outcomes.keys()))

        # Set initial state if specified
        if self.start_state:
            root.set("start_state", self.start_state)

        # Filter to only include root-level states (not nested in containers)
        root_level_states = {
            name: node
            for name, node in self.state_nodes.items()
            if not hasattr(node, "parent_container") or node.parent_container is None
        }

        # Add states recursively
        self._save_states_to_xml(root, root_level_states)

        # Check if file_path ends with .xml
        if not file_path.lower().endswith(".xml"):
            file_path += ".xml"

        # Write to file with lxml pretty printing
        tree = ET.ElementTree(root)
        tree.write(file_path, encoding="utf-8", xml_declaration=True, pretty_print=True)

    def _save_states_to_xml(self, parent_elem, state_nodes_dict):
        """Recursively save states and their children to XML."""
        # Add states
        for state_name, state_node in state_nodes_dict.items():
            if state_node.is_concurrence:
                # Concurrence state
                cc_elem = ET.SubElement(parent_elem, "Concurrence")
                cc_elem.set("name", state_name)

                # Add default outcome if available
                if hasattr(state_node, "default_outcome") and state_node.default_outcome:
                    cc_elem.set("default_outcome", state_node.default_outcome)

                # Get outcomes from final_outcomes (container children)
                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    cc_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:  # Skip None values
                            remap_elem = ET.SubElement(cc_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                # Recursively add child states if any
                if hasattr(state_node, "child_states") and state_node.child_states:
                    self._save_states_to_xml(cc_elem, state_node.child_states)

                # Add transitions for this concurrence
                self._save_transitions(cc_elem, state_node)

                # Add transitions from final outcomes inside this concurrence
                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    for outcome_node in state_node.final_outcomes.values():
                        self._save_transitions(cc_elem, outcome_node)

            elif state_node.is_state_machine:
                # Nested state machine
                sm_elem = ET.SubElement(parent_elem, "StateMachine")
                sm_elem.set("name", state_name)

                # Set initial state if the node has one
                if hasattr(state_node, "start_state") and state_node.start_state:
                    sm_elem.set("start_state", state_node.start_state)

                # Get outcomes from final_outcomes (container children)
                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    sm_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:  # Skip None values
                            remap_elem = ET.SubElement(sm_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                # Recursively add child states if any
                if hasattr(state_node, "child_states") and state_node.child_states:
                    self._save_states_to_xml(sm_elem, state_node.child_states)

                # Add transitions for this state machine
                self._save_transitions(sm_elem, state_node)

                # Add transitions from final outcomes inside this state machine
                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    for outcome_node in state_node.final_outcomes.values():
                        self._save_transitions(sm_elem, outcome_node)

            else:
                if state_node.plugin_info.plugin_type != "xml":
                    state_elem = ET.SubElement(parent_elem, "State")
                else:
                    state_elem = ET.SubElement(parent_elem, "StateMachine")
                state_elem.set("name", state_name)

                if state_node.plugin_info:
                    if state_node.plugin_info.plugin_type == "python":
                        state_elem.set("type", "py")
                        if state_node.plugin_info.module:
                            state_elem.set("module", state_node.plugin_info.module)
                        if state_node.plugin_info.class_name:
                            state_elem.set("class", state_node.plugin_info.class_name)
                    elif state_node.plugin_info.plugin_type == "cpp":
                        state_elem.set("type", "cpp")
                        if state_node.plugin_info.class_name:
                            state_elem.set("class", state_node.plugin_info.class_name)
                    elif state_node.plugin_info.plugin_type == "xml":
                        state_elem.set("type", "xml")
                        if state_node.plugin_info.file_name:
                            state_elem.set("file_name", state_node.plugin_info.file_name)
                            state_elem.set("package", state_node.plugin_info.package_name)

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:  # Skip None values
                            remap_elem = ET.SubElement(state_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                # Add transitions for this state
                self._save_transitions(state_elem, state_node)

    def _save_transitions(self, parent_elem, state_node):
        """Save transitions for a state node."""
        for connection in state_node.connections:
            if connection.from_node == state_node:
                transition = ET.SubElement(parent_elem, "Transition")
                if connection.outcome:  # Only add if outcome is not None
                    transition.set("from", connection.outcome)
                if (
                    connection.to_node and connection.to_node.name
                ):  # Only add if target exists
                    transition.set("to", connection.to_node.name)

    def load_from_xml(self, file_path: str):
        # Clear current state machine
        if not self.new_state_machine():
            return

        # Parse XML
        tree = ET.parse(file_path)
        root = tree.getroot()

        # Load root SM name
        sm_name = root.get("name", "")
        if sm_name:
            self.root_sm_name = sm_name
            self.root_sm_name_edit.setText(sm_name)

        # Load initial state
        start_state = root.get("start_state", "")

        # If no start_state attribute, find the first State element
        if not start_state:
            for elem in root:
                if elem.tag == "State":
                    start_state = elem.get("name", "")
                    break

        if start_state:
            self.start_state = start_state

        # Load states recursively with initial positioning
        self._load_states_from_xml(root, None)

        # Get final outcomes and create them BEFORE loading transitions
        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()
            # Create final outcomes at temporary positions
            # They will be repositioned after layout computation
            for i, outcome in enumerate(outcomes):
                y = 200 + i * 120
                node = FinalOutcomeNode(outcome, 800, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome] = node

        # Update initial state combo
        self.update_start_state_combo()

        # Set initial state selection
        if start_state:
            index = self.start_state_combo.findText(start_state)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)

        # Load transitions recursively BEFORE layout computation
        # This is CRITICAL - the layout algorithm needs the connection graph!
        self._load_transitions_from_xml(root, None)

        # NOW reorganize all containers to properly layout their children
        # Transitions are loaded, so the graph-based layout will work correctly
        self._reorganize_all_containers()

        # Force scene update to ensure all geometries are calculated
        self.canvas.scene.update()
        QApplication.processEvents()  # Process any pending events

        # Now reposition root-level elements based on actual sizes after auto-resize
        # This also positions final outcomes correctly based on their connections
        self._reposition_root_elements_after_resize()

        # Force update all connections throughout the entire scene
        # This ensures connections are drawn correctly both inside containers and at root level
        for state in self.state_nodes.values():
            if hasattr(state, "connections"):
                for conn in state.connections:
                    conn.update_position()

        for outcome in self.final_outcomes.values():
            if hasattr(outcome, "connections"):
                for conn in outcome.connections:
                    conn.update_position()

        # Multiple scene updates to ensure all connections are drawn correctly
        for _ in range(3):
            self.canvas.scene.update()
            QApplication.processEvents()

    def _reorganize_all_containers(self):
        """Reorganize all containers and their children after loading.
        Uses Sugiyama Framework for hierarchical graph layout.
        """
        # Reset layout RNG to ensure deterministic layout runs when a seed is set
        self._reset_layout_rng()
        # Process all containers (both root-level and nested)
        all_containers = []
        for node in self.state_nodes.values():
            if isinstance(node, ContainerStateNode):
                all_containers.append(node)

        # Sort containers by nesting depth (deepest first) to ensure
        # nested containers are processed before their parents
        def get_nesting_depth(container):
            depth = 0
            current = container
            while hasattr(current, "parent_container") and current.parent_container:
                depth += 1
                current = current.parent_container
            return depth

        all_containers.sort(key=get_nesting_depth, reverse=True)

        # Process each container with Sugiyama Framework
        for container in all_containers:
            self._layout_container_sugiyama(container)

        # Final pass to ensure all sizes are correct
        for container in all_containers:
            container.prepareGeometryChange()
            container.auto_resize_for_children()

        # Force all connections to update their positions
        # This is critical for containers to show their internal connections correctly
        for container in all_containers:
            # Ensure container geometry is up to date
            container.prepareGeometryChange()
            container.update()

            # Update all child connections
            for child_state in container.child_states.values():
                child_state.prepareGeometryChange()
                child_state.update()
                if hasattr(child_state, "connections"):
                    for conn in child_state.connections:
                        conn.update_position()

            for final_outcome in container.final_outcomes.values():
                final_outcome.prepareGeometryChange()
                final_outcome.update()
                if hasattr(final_outcome, "connections"):
                    for conn in final_outcome.connections:
                        conn.update_position()

        # Multiple geometry updates to finalize everything
        for _ in range(4):
            self.canvas.scene.update()
            QApplication.processEvents()

    def _layout_container_sugiyama(self, container: ContainerStateNode):
        """Layout children within a container using Force-Directed algorithm (Fruchterman-Reingold).

        Uses physical simulation with:
        - Repulsive forces between all nodes (prevent overlap)
        - Attractive forces along edges (keep connected nodes close)
        - Boundary forces (keep nodes within container)
        """
        if not container.child_states and not container.final_outcomes:
            return

        nodes = list(container.child_states.values())
        final_outcomes = list(container.final_outcomes.values())

        if not nodes:
            # Only final outcomes, position them simply
            rect = container.rect()
            y_start = rect.top() + 140
            for i, outcome in enumerate(final_outcomes):
                outcome.setPos(rect.left() + 120, y_start + i * 120)
            container.auto_resize_for_children()
            return

        # Build adjacency graph - INCLUDE final outcomes as terminal nodes
        all_nodes = nodes + final_outcomes
        graph = {node: [] for node in all_nodes}

        for node in nodes:
            # Safely access connections attribute
            if hasattr(node, "connections"):
                for conn in node.connections:
                    if conn.from_node == node:
                        # Include connections to both states and final outcomes
                        if conn.to_node in all_nodes:
                            graph[node].append(conn.to_node)

        # Use Force-Directed layout
        self._force_directed_layout(container, all_nodes, graph)

    def _force_directed_layout(self, container, nodes, graph):
        """Fruchterman-Reingold force-directed layout algorithm.
        
        Simulates physical forces:
        - Repulsion: All nodes repel each other (prevents overlap)
        - Attraction: Connected nodes attract each other (keeps graph connected)
        - Boundary: Nodes are pushed back if they leave the container
        
        Args:
            container: The container node
            nodes: List of all nodes to position
            graph: Adjacency dictionary mapping nodes to their neighbors
        """
        import math
        rng = self.layout_rng if getattr(self, "layout_rng", None) is not None else random
        
        rect = container.rect()
        
        # Layout parameters
        MARGIN_X = rect.width() * 0.05
        MARGIN_Y_TOP = rect.height() * 0.12
        MARGIN_Y_BOTTOM = rect.height() * 0.05
        
        area_width = rect.width() - (2 * MARGIN_X)
        area_height = rect.height() - MARGIN_Y_TOP - MARGIN_Y_BOTTOM
        
        # Get node dimensions
        node_dimensions = {}
        for node in nodes:
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                w, h = node.rect().width(), node.rect().height()
            else:
                w, h = node.boundingRect().width(), node.boundingRect().height()
            node_dimensions[node] = (w, h)
        
        # Initialize positions randomly or use current positions
        positions = {}
        for node in nodes:
            current_pos = node.pos()
            # Check if node already has a reasonable position
            if (rect.left() < current_pos.x() < rect.right() and 
                rect.top() < current_pos.y() < rect.bottom()):
                positions[node] = [current_pos.x(), current_pos.y()]
            else:
                # Random initial position
                x = rect.left() + MARGIN_X + rng.random() * area_width
                y = rect.top() + MARGIN_Y_TOP + rng.random() * area_height
                positions[node] = [x, y]
        
        # Algorithm parameters
        area = area_width * area_height
        k = math.sqrt(area / len(nodes))  # Optimal distance between nodes (increased for more separation)
        
        ITERATIONS = 100
        initial_temp = area_width / 10
        temp = initial_temp
        
        # Simulation loop
        for iteration in range(ITERATIONS):
            # Calculate displacement for each node
            displacement = {node: [0.0, 0.0] for node in nodes}
            
            # 1. Calculate repulsive forces between all pairs
            for i, node1 in enumerate(nodes):
                for node2 in nodes[i+1:]:
                    delta_x = positions[node1][0] - positions[node2][0]
                    delta_y = positions[node1][1] - positions[node2][1]
                    
                    # Distance between nodes
                    distance = math.sqrt(delta_x**2 + delta_y**2)
                    if distance < 0.01:
                        distance = 0.01  # Prevent division by zero
                    
                    # Repulsive force (inversely proportional to distance)
                    force = (k * k) / distance
                    
                    # Apply force
                    displacement[node1][0] += (delta_x / distance) * force
                    displacement[node1][1] += (delta_y / distance) * force
                    displacement[node2][0] -= (delta_x / distance) * force
                    displacement[node2][1] -= (delta_y / distance) * force
            
            # 2. Calculate attractive forces along edges
            for node in nodes:
                for neighbor in graph.get(node, []):
                    if neighbor not in nodes:
                        continue
                    
                    delta_x = positions[node][0] - positions[neighbor][0]
                    delta_y = positions[node][1] - positions[neighbor][1]
                    
                    distance = math.sqrt(delta_x**2 + delta_y**2)
                    if distance < 0.01:
                        distance = 0.01
                    
                    # Attractive force (proportional to distance)
                    force = (distance * distance) / k
                    
                    # Apply force
                    displacement[node][0] -= (delta_x / distance) * force
                    displacement[node][1] -= (delta_y / distance) * force
                    displacement[neighbor][0] += (delta_x / distance) * force
                    displacement[neighbor][1] += (delta_y / distance) * force
            
            # 3. Limit displacement by temperature and apply
            for node in nodes:
                disp_x = displacement[node][0]
                disp_y = displacement[node][1]
                
                disp_length = math.sqrt(disp_x**2 + disp_y**2)
                if disp_length < 0.01:
                    disp_length = 0.01
                
                # Limit by temperature
                limited_disp = min(disp_length, temp) / disp_length
                
                positions[node][0] += disp_x * limited_disp
                positions[node][1] += disp_y * limited_disp
                
                # Keep within boundaries
                node_w, node_h = node_dimensions[node]
                positions[node][0] = max(rect.left() + MARGIN_X, 
                                        min(positions[node][0], 
                                            rect.right() - MARGIN_X - node_w))
                positions[node][1] = max(rect.top() + MARGIN_Y_TOP, 
                                        min(positions[node][1], 
                                            rect.bottom() - MARGIN_Y_BOTTOM - node_h))
            
            # Cool down temperature
            temp = initial_temp * (1 - iteration / ITERATIONS)
        
        # Apply final positions
        for node, (x, y) in positions.items():
            node.setPos(x, y)
        
        container.auto_resize_for_children()

    def _preorder_outcomes_in_layers(self, layers, reverse_graph):
        """Pre-order final outcomes within their layers based on incoming connections.

        This helps establish a good initial ordering before the main crossing reduction,
        which is especially important for terminal nodes (outcomes) that may have
        multiple incoming edges from different layers.

        Uses a more sophisticated algorithm that considers:
        - Median position of predecessors
        - Layer proximity (closer layers weighted more)
        - Number of connections from each predecessor
        """
        from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

        for layer_idx, layer in enumerate(layers):
            # Find outcomes in this layer
            outcomes_in_layer = [
                node for node in layer if isinstance(node, FinalOutcomeNode)
            ]

            if len(outcomes_in_layer) <= 1:
                continue

            # Calculate weighted average position of incoming connections for each outcome
            outcome_positions = []

            for outcome in outcomes_in_layer:
                predecessors = reverse_graph.get(outcome, [])

                if predecessors:
                    # Find the layer indices and positions of predecessors
                    predecessor_info = []

                    for pred in predecessors:
                        # Find which layer and position the predecessor is in
                        for pred_layer_idx, pred_layer in enumerate(layers):
                            if pred in pred_layer:
                                pos_in_layer = pred_layer.index(pred)
                                # Weight by layer proximity (closer layers matter more)
                                layer_distance = abs(layer_idx - pred_layer_idx)
                                weight = 1.0 / (1.0 + layer_distance)
                                predecessor_info.append(
                                    (pos_in_layer, weight, pred_layer_idx)
                                )
                                break

                    if predecessor_info:
                        # Use weighted median of predecessor positions
                        # Sort by position
                        predecessor_info.sort(key=lambda x: x[0])

                        # Calculate total weight
                        total_weight = sum(info[1] for info in predecessor_info)

                        # Find weighted median
                        cumulative_weight = 0
                        weighted_median = 0

                        for pos, weight, _ in predecessor_info:
                            cumulative_weight += weight
                            if cumulative_weight >= total_weight / 2.0:
                                weighted_median = pos
                                break

                        avg_pos = weighted_median
                    else:
                        avg_pos = layer.index(outcome)  # Keep current position
                else:
                    # No predecessors, use current position
                    avg_pos = layer.index(outcome)

                outcome_positions.append((avg_pos, outcome))

            # Sort outcomes by their weighted average predecessor position
            outcome_positions.sort(key=lambda x: x[0])

            # Reorder outcomes in the layer
            # First, remove outcomes from layer
            for outcome in outcomes_in_layer:
                layer.remove(outcome)

            # Find where outcomes should be inserted (at the end of non-outcome nodes)
            non_outcome_count = len(layer)

            # Insert sorted outcomes
            for _, outcome in outcome_positions:
                layer.append(outcome)

    def _sugiyama_layer_assignment(self, nodes, graph, reverse_graph, start_state_name):
        """Step 1 of Sugiyama Framework: Assign layers to nodes using longest path layering.

        This ensures:
        - All edges point downward (from lower layer number to higher)
        - Start state is in layer 0
        - Each node is placed as far down as possible (longest path from start)
        """
        from collections import deque

        # Initialize all nodes at layer -1 (unassigned)
        layer_map = {node: -1 for node in nodes}

        # Find start node - prioritize exact name match
        start_node = None
        if start_state_name:
            # Look for exact match first
            for node in nodes:
                if node.name == start_state_name:
                    start_node = node
                    break

            # If no exact match, try to find partial match (for nested states)
            if not start_node:
                for node in nodes:
                    if start_state_name in node.name or node.name in start_state_name:
                        start_node = node
                        break

        # If no start state, find node with no incoming edges
        if not start_node:
            has_incoming = set()
            for node in nodes:
                for neighbor in graph[node]:
                    if neighbor in nodes:
                        has_incoming.add(neighbor)

            candidates = [n for n in nodes if n not in has_incoming]
            start_node = candidates[0] if candidates else nodes[0]

        # Use longest path layering algorithm with proper BFS
        layer_map[start_node] = 0
        queue = deque([start_node])
        visited = set([start_node])

        while queue:
            node = queue.popleft()
            current_layer = layer_map[node]

            # Process all outgoing edges
            for neighbor in graph[node]:
                if neighbor in nodes:
                    # Assign neighbor to next layer if not already assigned to a deeper layer
                    new_layer = current_layer + 1
                    if layer_map[neighbor] < new_layer:
                        layer_map[neighbor] = new_layer

                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

        # Handle nodes not reachable from start
        # These should be placed in appropriate layers based on their incoming edges
        unvisited = [n for n in nodes if layer_map[n] == -1]

        for node in unvisited:
            # If has incoming edges from visited nodes, place after them
            max_predecessor_layer = -1
            for predecessor in reverse_graph.get(node, []):
                if layer_map[predecessor] != -1:
                    max_predecessor_layer = max(
                        max_predecessor_layer, layer_map[predecessor]
                    )

            if max_predecessor_layer != -1:
                layer_map[node] = max_predecessor_layer + 1
            else:
                # No predecessors, place in layer 0
                layer_map[node] = 0

        # Optimize final outcome placement - place them based on median of their predecessors
        # This reduces crossing by positioning outcomes near their primary input sources
        from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

        max_layer = None

        for node in nodes:
            if isinstance(node, FinalOutcomeNode):
                predecessors = reverse_graph.get(node, [])
                if predecessors:
                    # Get layers of all predecessors
                    pred_layers = [
                        layer_map[pred]
                        for pred in predecessors
                        if pred in layer_map and layer_map[pred] != -1
                    ]
                    if pred_layers:
                        # Place outcome one layer after the maximum predecessor layer
                        # This ensures it's reachable from all predecessors
                        pred_layers.sort()
                        max_pred_layer = max(pred_layers)
                        optimal_layer = max_pred_layer + 1
                        # Update if this is better than current assignment
                        if layer_map[node] == -1 or layer_map[node] < optimal_layer:
                            layer_map[node] = optimal_layer
                            # Update max_layer if needed
                            if max_layer is None:
                                max_layer = optimal_layer
                            else:
                                max_layer = max(max_layer, optimal_layer)

        # Group nodes by layer
        max_layer = max(layer_map.values()) if layer_map else 0
        layers = [[] for _ in range(max_layer + 1)]

        for node, layer in layer_map.items():
            layers[layer].append(node)

        # Initial ordering within each layer based on dependencies
        # This provides a better starting point for crossing reduction
        from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

        for layer_idx, layer in enumerate(layers):
            if len(layer) <= 1:
                continue

            # For outcomes, sort by the average position of their predecessors
            # This gives a much better initial ordering
            def get_predecessor_positions(node):
                if not isinstance(node, FinalOutcomeNode):
                    return []
                predecessors = reverse_graph.get(node, [])
                positions = []
                for pred in predecessors:
                    # Find predecessor's position in its layer
                    for prev_layer_idx, prev_layer in enumerate(layers[:layer_idx]):
                        if pred in prev_layer:
                            positions.append((prev_layer_idx, prev_layer.index(pred)))
                            break
                return positions

            # Create a sorting key that considers:
            # 1. Whether it's an outcome (outcomes last)
            # 2. Average layer and position of predecessors
            # 3. Number of predecessors (fewer first for non-outcomes)
            # 4. Node name for stability
            def ordering_key(node):
                is_outcome = isinstance(node, FinalOutcomeNode)
                num_predecessors = len(reverse_graph.get(node, []))

                # Calculate average predecessor position
                pred_positions = get_predecessor_positions(node)
                if pred_positions:
                    # Weight by layer (closer layers = higher weight)
                    weighted_sum = sum(
                        (layer_idx + 1) * pos for layer_idx, pos in pred_positions
                    )
                    weighted_count = sum(layer_idx + 1 for layer_idx, _ in pred_positions)
                    avg_pred_pos = (
                        weighted_sum / weighted_count if weighted_count > 0 else 0
                    )
                else:
                    avg_pred_pos = (
                        0 if not is_outcome else 999
                    )  # Outcomes without preds go last

                return (is_outcome, avg_pred_pos, num_predecessors, node.name)

            layer.sort(key=ordering_key)

        return layers

    def _sugiyama_crossing_reduction(self, layers, graph, reverse_graph, iterations=100):
        """Step 2 of Sugiyama Framework: Reduce edge crossings using barycentric method.

        Uses median/barycentric heuristic to order nodes within each layer
        to minimize edge crossings between adjacent layers.
        """
        from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

        # Make a copy to avoid modifying original
        layers = [layer[:] for layer in layers]

        best_layers = None
        best_crossings = float("inf")
        no_improvement_count = 0
        MAX_NO_IMPROVEMENT = 10  # Stop if no improvement for 10 consecutive iterations

        for iteration in range(iterations):
            # Downward pass: order based on successors
            for i in range(len(layers) - 1):
                if len(layers[i]) > 1:
                    self._order_layer_by_barycenter(layers[i], layers[i + 1], graph)

            # Upward pass: order based on predecessors
            for i in range(len(layers) - 1, 0, -1):
                if len(layers[i]) > 1:
                    self._order_layer_by_barycenter(
                        layers[i], layers[i - 1], reverse_graph
                    )

            # Additional pass: Focus on layers containing outcomes
            # Outcomes often have multiple incoming edges, so extra ordering helps
            if iteration % 2 == 0:  # Every 2nd iteration
                for i in range(len(layers) - 1, 0, -1):
                    # Check if layer has outcomes
                    has_outcomes = any(
                        isinstance(node, FinalOutcomeNode) for node in layers[i]
                    )
                    if has_outcomes and len(layers[i]) > 1:
                        # Extra ordering pass for outcome-heavy layers
                        self._order_layer_by_barycenter(
                            layers[i], layers[i - 1], reverse_graph
                        )

            # Additional pass: Advanced local optimization every 4 iterations
            if iteration % 4 == 0 and iteration > 0:
                for layer_idx, layer in enumerate(layers):
                    if len(layer) > 2:
                        self._local_crossing_optimization(
                            layer, layers, layer_idx, graph, reverse_graph
                        )

            # Count crossings and keep track of best solution
            crossings = self._count_crossings(layers, graph)
            if crossings < best_crossings:
                best_crossings = crossings
                best_layers = [layer[:] for layer in layers]
                no_improvement_count = 0
            else:
                no_improvement_count += 1

            # Early exit if no crossings
            if crossings == 0:
                break

            # Early exit if no improvement for several iterations
            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                break

        return best_layers if best_layers else layers

    def _count_crossings(self, layers, graph):
        """Count the total number of edge crossings between adjacent layers."""
        total_crossings = 0

        for i in range(len(layers) - 1):
            layer1 = layers[i]
            layer2 = layers[i + 1]

            # Create position maps
            pos1 = {node: idx for idx, node in enumerate(layer1)}
            pos2 = {node: idx for idx, node in enumerate(layer2)}

            # Get all edges between these two layers
            edges = []
            for node in layer1:
                for neighbor in graph.get(node, []):
                    if neighbor in pos2:
                        edges.append((pos1[node], pos2[neighbor]))

            # Count crossings between all pairs of edges
            for i in range(len(edges)):
                for j in range(i + 1, len(edges)):
                    # Two edges (a1, a2) and (b1, b2) cross if:
                    # (a1 < b1 and a2 > b2) or (a1 > b1 and a2 < b2)
                    a1, a2 = edges[i]
                    b1, b2 = edges[j]
                    if (a1 < b1 and a2 > b2) or (a1 > b1 and a2 < b2):
                        total_crossings += 1

        return total_crossings

    def _local_crossing_optimization(
        self, layer, layers, layer_idx, graph, reverse_graph
    ):
        """Apply local optimization by trying adjacent swaps to reduce crossings.

        This greedy local search tries swapping adjacent nodes to find immediate
        crossing reductions.
        """
        if len(layer) < 2:
            return

        improved = True
        max_passes = 3
        pass_count = 0

        while improved and pass_count < max_passes:
            improved = False
            pass_count += 1

            for i in range(len(layer) - 1):
                # Try swapping nodes at positions i and i+1
                layer[i], layer[i + 1] = layer[i + 1], layer[i]

                # Count crossings before and after
                crossings_after = self._count_crossings(layers, graph)

                # If swap improved things, keep it
                # Otherwise, swap back
                # We use a simple greedy approach here
                # In practice, we'd need to store the before state, but this is a local optimization
                # so we just try and see if the overall structure is better

                # For simplicity, always try and keep the swap that reduces local crossings
                # between this layer and adjacent layers
                local_crossings = 0

                # Count crossings with previous layer
                if layer_idx > 0:
                    local_crossings += self._count_layer_crossings(
                        layers[layer_idx - 1], layer, reverse_graph
                    )

                # Count crossings with next layer
                if layer_idx < len(layers) - 1:
                    local_crossings += self._count_layer_crossings(
                        layer, layers[layer_idx + 1], graph
                    )

                # Swap back to test original
                layer[i], layer[i + 1] = layer[i + 1], layer[i]

                local_crossings_before = 0
                if layer_idx > 0:
                    local_crossings_before += self._count_layer_crossings(
                        layers[layer_idx - 1], layer, reverse_graph
                    )
                if layer_idx < len(layers) - 1:
                    local_crossings_before += self._count_layer_crossings(
                        layer, layers[layer_idx + 1], graph
                    )

                # If swapping reduced crossings, keep the swap
                if local_crossings < local_crossings_before:
                    layer[i], layer[i + 1] = layer[i + 1], layer[i]
                    improved = True

    def _count_layer_crossings(self, layer1, layer2, edge_map):
        """Count crossings between two specific layers."""
        crossings = 0

        pos1 = {node: idx for idx, node in enumerate(layer1)}
        pos2 = {node: idx for idx, node in enumerate(layer2)}

        edges = []
        for node in layer1:
            for neighbor in edge_map.get(node, []):
                if neighbor in pos2:
                    edges.append((pos1[node], pos2[neighbor]))

        for i in range(len(edges)):
            for j in range(i + 1, len(edges)):
                a1, a2 = edges[i]
                b1, b2 = edges[j]
                if (a1 < b1 and a2 > b2) or (a1 > b1 and a2 < b2):
                    crossings += 1

        return crossings

    def _order_layer_by_barycenter(self, current_layer, adjacent_layer, edge_map):
        """Order nodes in current_layer based on barycenter positions in adjacent_layer.

        Uses weighted median heuristic for optimal ordering to minimize crossings.
        The weighted median gives better results than simple median for nodes with
        multiple connections.
        """
        from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode

        # Create position map for adjacent layer
        pos_map = {node: idx for idx, node in enumerate(adjacent_layer)}

        # Calculate barycenter for each node in current layer
        barycenters = []
        for node_idx, node in enumerate(current_layer):
            neighbors = edge_map.get(node, [])
            neighbor_positions = [
                pos_map[neighbor] for neighbor in neighbors if neighbor in pos_map
            ]

            if neighbor_positions:
                # Use WEIGHTED MEDIAN for better results
                # For final outcomes with many predecessors, this gives better positioning
                neighbor_positions.sort()
                n = len(neighbor_positions)

                if isinstance(node, FinalOutcomeNode) and n > 2:
                    # For outcomes with many connections, use a weighted approach
                    # Give more weight to central positions to avoid extreme placements
                    weights = []
                    for i, pos in enumerate(neighbor_positions):
                        # Central positions get higher weight
                        distance_from_center = abs(i - n / 2.0)
                        weight = 1.0 / (1.0 + distance_from_center * 0.5)
                        weights.append(weight)

                    # Calculate weighted median
                    total_weight = sum(weights)
                    weighted_positions = [
                        (neighbor_positions[i], weights[i]) for i in range(n)
                    ]

                    cumulative_weight = 0
                    for pos, weight in weighted_positions:
                        cumulative_weight += weight
                        if cumulative_weight >= total_weight / 2.0:
                            barycenter = pos
                            break
                    else:
                        barycenter = neighbor_positions[n // 2]
                else:
                    # Standard median for regular nodes
                    if n % 2 == 1:
                        # Odd number: take middle element
                        barycenter = neighbor_positions[n // 2]
                    else:
                        # Even number: take average of two middle elements
                        barycenter = (
                            neighbor_positions[n // 2 - 1] + neighbor_positions[n // 2]
                        ) / 2.0
            else:
                # No neighbors, keep relative position
                # Use current position normalized to middle of layer
                barycenter = (
                    len(current_layer) / 2.0
                    + (node_idx - len(current_layer) / 2.0) * 0.01
                )

            barycenters.append((barycenter, node_idx, node))

        # Sort by barycenter value, with tie-breaking by original position
        barycenters.sort(key=lambda x: (x[0], x[1]))

        # Update layer order
        for i, (_, _, node) in enumerate(barycenters):
            current_layer[i] = node

    def _sugiyama_coordinate_assignment(
        self, container: ContainerStateNode, layers, graph, reverse_graph
    ):
        """Step 3 of Sugiyama Framework: Assign coordinates to nodes.

        Uses a grid-based layout with optimal spacing:
        - Nodes are positioned in a grid to maximize space usage
        - Considers actual node dimensions for precise placement
        - Minimizes edge crossings through strategic positioning
        - Dynamic spacing based on connection density

        Args:
            container: The container state node
            layers: List of layers, each containing nodes
            graph: Forward adjacency graph (node -> list of successors)
            reverse_graph: Reverse adjacency graph (node -> list of predecessors)
        """
        rect = container.rect()

        # Calculate dimensions for each node FIRST
        node_dimensions = {}
        max_node_height = 0
        
        for layer in layers:
            for node in layer:
                if isinstance(node, ContainerStateNode):
                    node.prepareGeometryChange()
                    width = node.rect().width()
                    height = node.rect().height()
                else:
                    width = node.boundingRect().width()
                    height = node.boundingRect().height()
                node_dimensions[node] = (width, height)
                max_node_height = max(max_node_height, height)
        
        # Dynamic margins based on container size (percentage-based)
        MARGIN_RATIO_X = 0.01  # 3% margin on each side
        MARGIN_RATIO_Y_TOP = 0.01  # 12% margin top (for header)
        MARGIN_RATIO_Y_BOTTOM = 0.01  # 3% margin bottom
        
        margin_x = rect.width() * MARGIN_RATIO_X
        margin_y_top = rect.height() * MARGIN_RATIO_Y_TOP
        margin_y_bottom = rect.height() * MARGIN_RATIO_Y_BOTTOM
        
        # Use local coordinates relative to rect origin
        START_X = rect.left() + margin_x
        START_Y = rect.top() + margin_y_top
        
        # Calculate available space
        available_width = rect.width() - (2 * margin_x)
        available_height = rect.height() - margin_y_top - margin_y_bottom
        
        num_layers = len(layers)
        
        # Calculate ADAPTIVE vertical spacing
        if num_layers > 1:
            # Space available after accounting for node heights
            total_nodes_height = num_layers * max_node_height
            space_for_gaps = available_height - total_nodes_height
            layer_spacing = space_for_gaps / (num_layers - 1) if space_for_gaps > 0 else max_node_height * 0.3
        else:
            layer_spacing = 0

        # Phase 1: Calculate layer Y positions with adaptive spacing
        layer_y_positions = []
        current_y = START_Y
        
        for layer_idx in range(num_layers):
            layer_y_positions.append(current_y)
            if layer_idx < num_layers - 1:
                # Add node height + calculated spacing
                current_y += max_node_height + layer_spacing

        # Phase 2: Position nodes with adaptive horizontal spacing
        positions = {}
        
        for layer_idx, layer in enumerate(layers):
            layer_y = layer_y_positions[layer_idx]
            
            if len(layer) == 0:
                continue
            
            # Calculate total width of all nodes in this layer
            total_node_width = sum(node_dimensions[node][0] for node in layer)
            
            if len(layer) == 1:
                # Single node - center it horizontally
                node = layer[0]
                node_width = node_dimensions[node][0]
                x = START_X + (available_width - node_width) / 2
                positions[node] = (x, layer_y)
            else:
                # Multiple nodes - distribute evenly from left to right edge
                # Use edge-to-edge distribution (no extra gaps at edges)
                available_space = available_width - total_node_width
                
                if len(layer) > 1:
                    # Distribute space between nodes only
                    gap_size = available_space / (len(layer) - 1)
                else:
                    gap_size = 0
                
                # Start at left edge
                current_x = START_X
                
                for node in layer:
                    positions[node] = (current_x, layer_y)
                    width = node_dimensions[node][0]
                    current_x += width + gap_size

        # Phase 3: Optimize positions using median of connected nodes
        for iteration in range(5):
            for layer_idx in range(len(layers)):
                layer = layers[layer_idx]
                
                if len(layer) <= 1:
                    continue
                
                # Calculate ideal X based on connections
                for node in layer:
                    connected_positions = []
                    
                    # Predecessors
                    for pred in reverse_graph.get(node, []):
                        if pred in positions:
                            connected_positions.append(positions[pred][0])
                    
                    # Successors
                    for succ in graph.get(node, []):
                        if succ in positions:
                            connected_positions.append(positions[succ][0])
                    
                    if connected_positions:
                        connected_positions.sort()
                        median_x = connected_positions[len(connected_positions) // 2]
                        
                        current_x, current_y = positions[node]
                        new_x = current_x + (median_x - current_x) * 0.3
                        positions[node] = (new_x, current_y)

        # Calculate minimum spacing for collision detection
        # Use the smallest gap size as the minimum spacing
        min_spacing = min(
            available_width / (max(len(layer) for layer in layers) + 1),
            layer_spacing if layer_spacing > 0 else max_node_height * 0.3
        )
        min_spacing = max(20, min_spacing)  # At least 20px, but use calculated value

        # Phase 4: Resolve collisions with grid-aware algorithm
        positions = self._resolve_collisions_grid_based(positions, node_dimensions, min_spacing)

        # Phase 5: Minimize crossings with grid awareness
        positions = self._minimize_crossings_grid_aware(
            positions, node_dimensions, layers, graph, reverse_graph, min_spacing
        )

        # Phase 6: Apply positions
        for node, (x, y) in positions.items():
            node.setPos(x, y)

        container.auto_resize_for_children()

    def _resolve_collisions_grid_based(self, positions, node_dimensions, min_spacing):
        """Resolve collisions using grid-aware approach that maintains layer structure.
        
        Args:
            positions: Dict mapping nodes to (x, y) tuples
            node_dimensions: Dict mapping nodes to (width, height) tuples
            min_spacing: Minimum spacing between nodes
            
        Returns:
            Updated positions dict with no overlaps
        """
        MAX_ITERATIONS = 30
        PUSH_DISTANCE = 40  # Aumentado para más separación
        
        for iteration in range(MAX_ITERATIONS):
            collision_found = False
            adjusted_positions = positions.copy()
            
            # Group nodes by Y position (layers)
            layers_dict = {}
            for node, (x, y) in positions.items():
                if y not in layers_dict:
                    layers_dict[y] = []
                layers_dict[y].append(node)
            
            # Check collisions within each layer
            for y_pos, layer_nodes in layers_dict.items():
                # Sort by X position
                layer_nodes.sort(key=lambda n: positions[n][0])
                
                # Check adjacent nodes in same layer
                for i in range(len(layer_nodes) - 1):
                    node1 = layer_nodes[i]
                    node2 = layer_nodes[i + 1]
                    
                    x1, y1 = positions[node1]
                    x2, y2 = positions[node2]
                    w1, h1 = node_dimensions[node1]
                    w2, h2 = node_dimensions[node2]
                    
                    # Check horizontal overlap
                    if x1 + w1 + min_spacing > x2:
                        collision_found = True
                        # Push node2 to the right
                        needed_space = (x1 + w1 + min_spacing) - x2
                        adjusted_positions[node2] = (x2 + needed_space, y2)
            
            positions = adjusted_positions
            
            if not collision_found:
                break
        
        return positions

    def _minimize_crossings_grid_aware(self, positions, node_dimensions, layers, graph, reverse_graph, base_spacing):
        """Minimize edge crossings while maintaining grid structure.
        
        Uses layer-by-layer optimization to reduce crossings without causing collisions.
        
        Args:
            positions: Current node positions
            node_dimensions: Node dimensions
            layers: Layer structure
            graph: Forward adjacency graph
            reverse_graph: Reverse adjacency graph
            base_spacing: Base spacing between nodes
            
        Returns:
            Optimized positions
        """
        # Try small adjustments within each layer to reduce crossings
        for iteration in range(10):
            improved = False
            
            for layer in layers:
                if len(layer) <= 1:
                    continue
                
                # Try swapping adjacent nodes if it reduces crossings
                for i in range(len(layer) - 1):
                    node1 = layer[i]
                    node2 = layer[i + 1]
                    
                    # Count crossings before swap
                    crossings_before = self._count_node_crossings(node1, positions, graph, reverse_graph)
                    crossings_before += self._count_node_crossings(node2, positions, graph, reverse_graph)
                    
                    # Swap positions
                    pos1 = positions[node1]
                    pos2 = positions[node2]
                    positions[node1] = pos2
                    positions[node2] = pos1
                    
                    # Count crossings after swap
                    crossings_after = self._count_node_crossings(node1, positions, graph, reverse_graph)
                    crossings_after += self._count_node_crossings(node2, positions, graph, reverse_graph)
                    
                    # Keep swap if it reduces crossings
                    if crossings_after < crossings_before:
                        improved = True
                    else:
                        # Revert swap
                        positions[node1] = pos1
                        positions[node2] = pos2
            
            if not improved:
                break
        
        return positions

    def _count_node_crossings(self, node, positions, graph, reverse_graph):
        """Count crossings for edges connected to a specific node.
        
        Args:
            node: Node to check
            positions: All node positions
            graph: Forward graph
            reverse_graph: Reverse graph
            
        Returns:
            Number of crossings for this node's edges
        """
        crossings = 0
        node_edges = []
        
        # Get all edges for this node
        if node in positions:
            x1, y1 = positions[node]
            
            # Outgoing edges
            for succ in graph.get(node, []):
                if succ in positions:
                    x2, y2 = positions[succ]
                    node_edges.append(((x1, y1), (x2, y2)))
            
            # Incoming edges
            for pred in reverse_graph.get(node, []):
                if pred in positions:
                    x0, y0 = positions[pred]
                    node_edges.append(((x0, y0), (x1, y1)))
        
        # Count crossings with all other edges
        all_edges = []
        for n, neighbors in graph.items():
            if n not in positions:
                continue
            nx, ny = positions[n]
            for neighbor in neighbors:
                if neighbor not in positions:
                    continue
                nnx, nny = positions[neighbor]
                all_edges.append(((nx, ny), (nnx, nny)))
        
        # Check each node edge against all edges
        for edge1 in node_edges:
            for edge2 in all_edges:
                if edge1 != edge2 and self._edges_cross(edge1, edge2):
                    crossings += 1
        
        return crossings

    def _resolve_collisions(self, positions, node_dimensions, min_spacing):
        """Detect and resolve overlapping nodes by adjusting positions.
        
        Uses iterative collision detection with spatial hashing for efficiency.
        Adjusts positions horizontally to maintain layer structure while preventing overlaps.
        
        Args:
            positions: Dict mapping nodes to (x, y) tuples
            node_dimensions: Dict mapping nodes to (width, height) tuples
            min_spacing: Minimum spacing between nodes
            
        Returns:
            Updated positions dict with no overlaps
        """
        MAX_ITERATIONS = 50
        PUSH_DISTANCE = 20  # Distance to push nodes apart per iteration
        
        for iteration in range(MAX_ITERATIONS):
            collision_found = False
            adjusted_positions = positions.copy()
            
            # Check all pairs for collisions
            nodes = list(positions.keys())
            for i in range(len(nodes)):
                for j in range(i + 1, len(nodes)):
                    node1, node2 = nodes[i], nodes[j]
                    x1, y1 = positions[node1]
                    x2, y2 = positions[node2]
                    w1, h1 = node_dimensions[node1]
                    w2, h2 = node_dimensions[node2]
                    
                    # Check if bounding boxes overlap (with min_spacing buffer)
                    if (abs(x1 - x2) < (w1 + w2) / 2 + min_spacing and
                        abs(y1 - y2) < (h1 + h2) / 2 + min_spacing):
                        
                        collision_found = True
                        
                        # Calculate overlap amounts
                        x_overlap = (w1 + w2) / 2 + min_spacing - abs(x1 - x2)
                        y_overlap = (h1 + h2) / 2 + min_spacing - abs(y1 - y2)
                        
                        # Push nodes apart in the direction with less overlap
                        # Prefer horizontal separation to maintain layer structure
                        if x_overlap < y_overlap * 1.5:  # Bias towards horizontal
                            # Push horizontally
                            if x1 < x2:
                                adjusted_positions[node1] = (x1 - PUSH_DISTANCE, y1)
                                adjusted_positions[node2] = (x2 + PUSH_DISTANCE, y2)
                            else:
                                adjusted_positions[node1] = (x1 + PUSH_DISTANCE, y1)
                                adjusted_positions[node2] = (x2 - PUSH_DISTANCE, y2)
                        else:
                            # Push vertically
                            if y1 < y2:
                                adjusted_positions[node1] = (x1, y1 - PUSH_DISTANCE)
                                adjusted_positions[node2] = (x2, y2 + PUSH_DISTANCE)
                            else:
                                adjusted_positions[node1] = (x1, y1 + PUSH_DISTANCE)
                                adjusted_positions[node2] = (x2, y2 - PUSH_DISTANCE)
            
            positions = adjusted_positions
            
            # If no collisions found, we're done
            if not collision_found:
                break
        
        return positions

    def _minimize_edge_crossings_final(self, positions, node_dimensions, layers, graph, reverse_graph):
        """Final pass to minimize edge crossings through micro-adjustments.
        
        Makes small position adjustments to reduce crossings without causing overlaps.
        Uses simulated annealing approach for global optimization.
        
        Args:
            positions: Current node positions
            node_dimensions: Node dimensions
            layers: Layer structure
            graph: Forward adjacency graph
            reverse_graph: Reverse adjacency graph
            
        Returns:
            Optimized positions
        """
        import math
        rng = self.layout_rng if getattr(self, "layout_rng", None) is not None else random
        
        MAX_ITERATIONS = 100
        INITIAL_TEMP = 50.0
        COOLING_RATE = 0.95
        MAX_MOVE = 30  # Maximum movement per iteration
        
        current_positions = positions.copy()
        best_positions = positions.copy()
        current_crossings = self._count_all_crossings(current_positions, graph)
        best_crossings = current_crossings
        
        temperature = INITIAL_TEMP
        
        for iteration in range(MAX_ITERATIONS):
            # Try moving a random node
            node = rng.choice(list(current_positions.keys()))
            old_pos = current_positions[node]
            
            # Generate small random movement
            dx = rng.uniform(-MAX_MOVE, MAX_MOVE)
            dy = rng.uniform(-MAX_MOVE, MAX_MOVE)
            new_pos = (old_pos[0] + dx, old_pos[1] + dy)
            
            # Apply tentative move
            current_positions[node] = new_pos
            
            # Check if this creates collisions
            has_collision = self._check_collision_at_position(
                node, new_pos, current_positions, node_dimensions, min_spacing=100
            )
            
            if not has_collision:
                # Calculate new crossing count
                new_crossings = self._count_all_crossings(current_positions, graph)
                
                # Decide whether to accept the move (simulated annealing)
                delta = new_crossings - current_crossings
                
                if delta < 0 or rng.random() < math.exp(-delta / temperature):
                    # Accept move
                    current_crossings = new_crossings
                    
                    if new_crossings < best_crossings:
                        best_crossings = new_crossings
                        best_positions = current_positions.copy()
                else:
                    # Reject move
                    current_positions[node] = old_pos
            else:
                # Reject move due to collision
                current_positions[node] = old_pos
            
            # Cool down
            temperature *= COOLING_RATE
            
            # Early exit if no crossings
            if best_crossings == 0:
                break
        
        return best_positions

    def _check_collision_at_position(self, node, pos, positions, node_dimensions, min_spacing):
        """Check if a node at a given position would collide with others.
        
        Args:
            node: Node to check
            pos: Position tuple (x, y)
            positions: Dict of all node positions
            node_dimensions: Dict of node dimensions
            min_spacing: Minimum required spacing
            
        Returns:
            True if collision detected, False otherwise
        """
        x, y = pos
        w, h = node_dimensions[node]
        
        for other_node, other_pos in positions.items():
            if other_node == node:
                continue
            
            ox, oy = other_pos
            ow, oh = node_dimensions[other_node]
            
            # Check bounding box overlap with spacing
            if (abs(x - ox) < (w + ow) / 2 + min_spacing and
                abs(y - oy) < (h + oh) / 2 + min_spacing):
                return True

        # Helper to compute rect from center coords
        def rect_from_center(cx, cy, width, height):
            left = cx - width / 2.0
            right = cx + width / 2.0
            top = cy - height / 2.0
            bottom = cy + height / 2.0
            return left, top, right, bottom

        # Helper to detect if a segment intersects a rectangle
        def segment_intersects_rect(x1, y1, x2, y2, left, top, right, bottom):
            # Quick reject: both points to one side
            if (x1 < left and x2 < left) or (x1 > right and x2 > right) or (
                y1 < top and y2 < top) or (y1 > bottom and y2 > bottom
            ):
                return False

            # Check if either endpoint is inside the rect
            if left <= x1 <= right and top <= y1 <= bottom:
                return True
            if left <= x2 <= right and top <= y2 <= bottom:
                return True

            # Check intersection with each rectangle edge (as segments)
            def seg_intersect(ax, ay, bx, by, cx, cy, dx, dy):
                def orientation(px, py, qx, qy, rx, ry):
                    val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy)
                    if abs(val) < 1e-9:
                        return 0
                    return 1 if val > 0 else 2

                o1 = orientation(ax, ay, bx, by, cx, cy)
                o2 = orientation(ax, ay, bx, by, dx, dy)
                o3 = orientation(cx, cy, dx, dy, ax, ay)
                o4 = orientation(cx, cy, dx, dy, bx, by)

                if o1 != o2 and o3 != o4:
                    return True
                return False

            # Rectangle edges
            # Left edge
            if seg_intersect(x1, y1, x2, y2, left, top, left, bottom):
                return True
            # Right edge
            if seg_intersect(x1, y1, x2, y2, right, top, right, bottom):
                return True
            # Top edge
            if seg_intersect(x1, y1, x2, y2, left, top, right, top):
                return True
            # Bottom edge
            if seg_intersect(x1, y1, x2, y2, left, bottom, right, bottom):
                return True
            return False

        # Check box intersections with connection segments to avoid nodes sitting over other edges
        for conn in getattr(self, "connections", []):
            # Skip connections that attach to this node - self connections are allowed
            if conn.from_node == node or conn.to_node == node:
                continue

            # For simulation, use the positions dict centers (fallback to scene positions)
            if conn.from_node in positions and conn.to_node in positions:
                fx, fy = positions[conn.from_node]
                tx, ty = positions[conn.to_node]
            else:
                # Fall back to actual scene positions
                fc = conn.from_node.scenePos()
                tx = conn.to_node.scenePos().x()
                ty = conn.to_node.scenePos().y()
                fx = fc.x()
                fy = fc.y()

            # Node rect
            n_left, n_top, n_right, n_bottom = rect_from_center(x, y, w, h)

            # Check if segment intersects node rect (avoid crossing edges)
            if segment_intersects_rect(fx, fy, tx, ty, n_left, n_top, n_right, n_bottom):
                return True

            # Also check approx label region (midpoint area)
            midx = (fx + tx) / 2.0
            midy = (fy + ty) / 2.0
            # Estimate label size if available
            try:
                label_rect = conn.label.boundingRect()
                lw, lh = label_rect.width(), label_rect.height()
            except Exception:
                lw, lh = 80, 20
            l_left, l_top, l_right, l_bottom = rect_from_center(midx, midy, lw + 8, lh + 8)
            if not (n_right < l_left or n_left > l_right or n_bottom < l_top or n_top > l_bottom):
                return True

        # Check against container bounds/header to avoid placing node on top the header or outside container
        # If node is inside a container, ensure it doesn't overlap header (top area) and stays within bounds
        if hasattr(node, "parent_container") and node.parent_container:
            container = node.parent_container
            crect = container.rect()
            # Header/gap (header height from container.header rect)
            try:
                header_h = container.header.rect().height()
            except Exception:
                header_h = 50

            # Define safe area margins
            SAFE_LEFT = crect.left() + 10
            SAFE_RIGHT = crect.right() - 10
            SAFE_TOP = crect.top() + header_h + 10
            SAFE_BOTTOM = crect.bottom() - 10

            # Node boundaries
            node_left, node_top, node_right, node_bottom = rect_from_center(x, y, w, h)

            if node_left < SAFE_LEFT or node_right > SAFE_RIGHT or node_top < SAFE_TOP or node_bottom > SAFE_BOTTOM:
                return True

        # Prevent nodes from overlapping containers they are not inside (root nodes vs container boxes)
        for c in [n for n in self.state_nodes.values() if isinstance(n, ContainerStateNode)]:
            if hasattr(node, "parent_container") and node.parent_container == c:
                continue
            # For root-level nodes, ensure they don't overlap container boxes
            crect = c.rect()
            c_left = crect.left()
            c_right = crect.right()
            c_top = crect.top()
            c_bottom = crect.bottom()
            # Container position in the same coordinate space? Assume positions are comparable
            # Compute container center
            c_cx = c.scenePos().x() + (c_left + c_right) / 2.0
            c_cy = c.scenePos().y() + (c_top + c_bottom) / 2.0
            c_w = c.rect().width()
            c_h = c.rect().height()
            c_left = c_cx - c_w / 2.0
            c_right = c_cx + c_w / 2.0
            c_top = c_cy - c_h / 2.0
            c_bottom = c_cy + c_h / 2.0
            # Node world-space bounding box
            # For positions already in world-space (root nodes), pos is center
            node_left, node_top, node_right, node_bottom = rect_from_center(x, y, w, h)
            # If overlap -> collision
            if not (node_right < c_left or node_left > c_right or node_bottom < c_top or node_top > c_bottom):
                return True
        
        return False

    def _count_all_crossings(self, positions, graph):
        """Count total edge crossings across all connections.
        
        Args:
            positions: Node positions
            graph: Adjacency graph
            
        Returns:
            Total number of edge crossings
        """
        # Collect all edges with their positions
        edges = []
        for node, neighbors in graph.items():
            if node not in positions:
                continue
            x1, y1 = positions[node]
            for neighbor in neighbors:
                if neighbor not in positions:
                    continue
                x2, y2 = positions[neighbor]
                edges.append(((x1, y1), (x2, y2)))
        
        # Count crossings between all edge pairs
        crossings = 0
        for i in range(len(edges)):
            for j in range(i + 1, len(edges)):
                if self._edges_cross(edges[i], edges[j]):
                    crossings += 1
        
        return crossings

    def _edges_cross(self, edge1, edge2):
        """Check if two edges cross using line segment intersection.
        
        Args:
            edge1: Tuple of ((x1, y1), (x2, y2))
            edge2: Tuple of ((x3, y3), (x4, y4))
            
        Returns:
            True if edges cross, False otherwise
        """
        (x1, y1), (x2, y2) = edge1
        (x3, y3), (x4, y4) = edge2
        
        # Skip if edges share endpoints
        if (x1, y1) == (x3, y3) or (x1, y1) == (x4, y4) or \
           (x2, y2) == (x3, y3) or (x2, y2) == (x4, y4):
            return False
        
        # Calculate orientation of ordered triplet (p, q, r)
        def orientation(px, py, qx, qy, rx, ry):
            val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy)
            if abs(val) < 1e-9:
                return 0  # Collinear
            return 1 if val > 0 else 2  # Clockwise or Counterclockwise
        
        o1 = orientation(x1, y1, x2, y2, x3, y3)
        o2 = orientation(x1, y1, x2, y2, x4, y4)
        o3 = orientation(x3, y3, x4, y4, x1, y1)
        o4 = orientation(x3, y3, x4, y4, x2, y2)
        
        # General case: segments intersect if orientations differ
        if o1 != o2 and o3 != o4:
            return True
        
        return False

    def _compute_layers(self, nodes, graph, start_state_name):
        """Compute hierarchical layers for nodes using longest path layering.
        Places start state in layer 0, then assigns other nodes based on longest path.
        """
        from collections import deque

        # Initialize all nodes at layer 0
        layer_map = {node: 0 for node in nodes}

        # Find start node
        start_node = None
        if start_state_name:
            for node in nodes:
                if node.name == start_state_name:
                    start_node = node
                    break

        # If no start state, use first node or node with no incoming edges
        if not start_node:
            # Find nodes with no incoming edges
            has_incoming = set()
            for node in nodes:
                for neighbor in graph[node]:
                    has_incoming.add(neighbor)

            candidates = [n for n in nodes if n not in has_incoming]
            start_node = candidates[0] if candidates else nodes[0]

        # BFS to assign layers based on longest path from start
        visited = set()
        queue = deque([(start_node, 0)])

        while queue:
            node, layer = queue.popleft()

            if node in visited and layer_map[node] >= layer:
                continue

            visited.add(node)
            layer_map[node] = max(layer_map[node], layer)

            # Process neighbors
            for neighbor in graph[node]:
                if neighbor in nodes:  # Only process nodes in this container
                    queue.append((neighbor, layer + 1))

        # Group nodes by layer
        max_layer = max(layer_map.values()) if layer_map else 0
        layers = [[] for _ in range(max_layer + 1)]

        for node, layer in layer_map.items():
            layers[layer].append(node)

        # Sort nodes within each layer to minimize edge crossings
        for layer in layers:
            if len(layer) > 1:
                # Simple heuristic: sort by number of connections
                layer.sort(key=lambda n: len(graph[n]), reverse=True)

        return layers

    def _position_layers(self, container: ContainerStateNode, layers, final_outcomes):
        """Position nodes in layers with proper top-to-bottom spacing.
        Arranges all nodes in a single vertical column for clean layout.
        """
        rect = container.rect()

        # Configuration for clean vertical layout
        START_X = rect.left() + 80  # Left padding
        START_Y = rect.top() + 100  # Top padding (below header)
        NODE_SPACING = 200  # Vertical spacing between nodes

        # Flatten all layers into a single vertical column for top-to-bottom layout
        # Order nodes by their layer (topological order) for logical flow
        all_nodes = []
        for layer in layers:
            all_nodes.extend(layer)

        # Position each node vertically from top to bottom
        current_y = START_Y

        for node in all_nodes:
            # Get node height
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                height = node.rect().height()
            else:
                height = node.boundingRect().height()

            # Position the node
            node.setPos(START_X, current_y)

            # Move to next position
            current_y += height + NODE_SPACING

        # Position final outcomes to the right of all nodes
        if final_outcomes:
            # Find the maximum width of all nodes
            max_width = 0
            for node in all_nodes:
                if isinstance(node, ContainerStateNode):
                    node.prepareGeometryChange()
                    width = node.rect().width()
                else:
                    width = node.boundingRect().width()
                max_width = max(max_width, width)

            # Position outcomes to the right with generous spacing
            outcome_x = START_X + max_width + 250
            outcome_y = START_Y

            for outcome in final_outcomes:
                outcome.setPos(outcome_x, outcome_y)
                outcome_y += 200  # Vertical spacing between outcomes

        # Resize container to fit all children
        container.auto_resize_for_children()

    def _refine_positions_force_directed(self, layers, positions, graph, iterations=5):
        """Refine node positions using force-directed algorithm to minimize crossings."""
        # Force-directed adjustment within layers
        for iteration in range(iterations):
            for layer_idx, layer in enumerate(layers):
                if len(layer) <= 1:
                    continue

                # Calculate forces between nodes in the same layer
                forces = {node: 0 for node in layer}

                for i, node in enumerate(layer):
                    # 1. Repulsion from neighbors in same layer (prevent overlap)
                    for j, other in enumerate(layer):
                        if i != j:
                            y1 = positions[node][1]
                            y2 = positions[other][1]
                            dist = abs(y1 - y2)

                            min_distance = 150  # Minimum distance between nodes
                            if dist < min_distance:
                                # Push apart with stronger force
                                force = (min_distance - dist) * 0.5
                                if y1 < y2:
                                    forces[node] -= force
                                else:
                                    forces[node] += force

                    # 2. Attraction to connected nodes (minimize edge length)
                    if layer_idx > 0:
                        # Check predecessors
                        for prev_node in layers[layer_idx - 1]:
                            if node in graph.get(prev_node, []):
                                y_prev = positions[prev_node][1]
                                y_curr = positions[node][1]
                                # Gentle attraction
                                forces[node] += (y_prev - y_curr) * 0.2

                    if layer_idx < len(layers) - 1:
                        # Check successors
                        for next_node in graph.get(node, []):
                            if next_node in layers[layer_idx + 1]:
                                y_next = positions[next_node][1]
                                y_curr = positions[node][1]
                                # Gentle attraction
                                forces[node] += (y_next - y_curr) * 0.2

                # Apply forces with damping
                damping = 0.8 - (iteration * 0.1)  # Reduce force over iterations
                for node in layer:
                    x, y = positions[node]
                    new_y = y + forces[node] * damping
                    # Ensure nodes stay in reasonable bounds
                    new_y = max(new_y, 160)  # Don't go above container top
                    positions[node] = (x, new_y)
                    node.setPos(x, new_y)

    def _reposition_root_elements_after_resize(self):
        """Reposition root-level elements using Sugiyama Framework.
        This ensures optimal spacing and prevents overlapping.
        Includes final outcomes in the graph layout.
        """
        # Get all root-level nodes
        root_nodes = []
        for node in self.state_nodes.values():
            if not hasattr(node, "parent_container") or node.parent_container is None:
                root_nodes.append(node)

        # Get all root-level final outcomes
        root_final_outcomes = list(self.final_outcomes.values())

        if not root_nodes and not root_final_outcomes:
            return

        # Build adjacency graph - INCLUDE final outcomes as terminal nodes
        all_nodes = root_nodes + root_final_outcomes
        graph = {node: [] for node in all_nodes}
        reverse_graph = {node: [] for node in all_nodes}

        for node in root_nodes:
            # Safely access connections attribute
            if hasattr(node, "connections"):
                for conn in node.connections:
                    if conn.from_node == node:
                        # Include connections to both states and final outcomes
                        if conn.to_node in all_nodes:
                            graph[node].append(conn.to_node)
                            reverse_graph[conn.to_node].append(node)

        # Verify we have some connections
        total_edges = sum(len(neighbors) for neighbors in graph.values())
        if total_edges == 0 and len(all_nodes) > 1:
            # No connections found - fall back to simple positioning
            x_pos = 120
            y_pos = 120
            for node in root_nodes:
                node.setPos(x_pos, y_pos)
                y_pos += 200
            for outcome in root_final_outcomes:
                outcome.setPos(
                    x_pos + 600, 120 + root_final_outcomes.index(outcome) * 150
                )
            return

        # Apply Force-Directed layout for root level
        self._force_directed_layout_root(all_nodes, graph)

    def _force_directed_layout_root(self, nodes, graph):
        """Force-directed layout for root-level nodes.
        
        Uses Fruchterman-Reingold algorithm adapted for the root canvas.
        
        Args:
            nodes: List of all root-level nodes
            graph: Adjacency dictionary
        """
        import math
        rng = self.layout_rng if getattr(self, "layout_rng", None) is not None else random
        
        # Get viewport dimensions
        viewport_rect = self.canvas.viewport().rect()
        viewport_width = viewport_rect.width()
        viewport_height = viewport_rect.height()
        
        # Margins
        MARGIN_X = viewport_width * 0.1
        MARGIN_Y = viewport_height * 0.1
        
        area_width = viewport_width - (2 * MARGIN_X)
        area_height = viewport_height - (2 * MARGIN_Y)
        
        # Get node dimensions
        node_dimensions = {}
        for node in nodes:
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                w, h = node.rect().width(), node.rect().height()
            else:
                w, h = node.boundingRect().width(), node.boundingRect().height()
            node_dimensions[node] = (w, h)
        
        # Initialize positions
        positions = {}
        for node in nodes:
            current_pos = node.scenePos()
            # Use current position if reasonable, otherwise random
            if (MARGIN_X < current_pos.x() < viewport_width - MARGIN_X and 
                MARGIN_Y < current_pos.y() < viewport_height - MARGIN_Y):
                positions[node] = [current_pos.x(), current_pos.y()]
            else:
                x = MARGIN_X + rng.random() * area_width
                y = MARGIN_Y + rng.random() * area_height
                positions[node] = [x, y]
        
        # Algorithm parameters
        area = area_width * area_height
        k = math.sqrt(area / len(nodes)) * 1.5  # Optimal distance (increased for more separation)
        
        ITERATIONS = 150
        initial_temp = area_width / 8
        temp = initial_temp
        
        # Simulation
        for iteration in range(ITERATIONS):
            displacement = {node: [0.0, 0.0] for node in nodes}
            
            # Repulsive forces
            for i, node1 in enumerate(nodes):
                for node2 in nodes[i+1:]:
                    delta_x = positions[node1][0] - positions[node2][0]
                    delta_y = positions[node1][1] - positions[node2][1]
                    
                    distance = math.sqrt(delta_x**2 + delta_y**2)
                    if distance < 0.01:
                        distance = 0.01
                    
                    force = (k * k) / distance
                    
                    displacement[node1][0] += (delta_x / distance) * force
                    displacement[node1][1] += (delta_y / distance) * force
                    displacement[node2][0] -= (delta_x / distance) * force
                    displacement[node2][1] -= (delta_y / distance) * force
            
            # Attractive forces
            for node in nodes:
                for neighbor in graph.get(node, []):
                    if neighbor not in nodes:
                        continue
                    
                    delta_x = positions[node][0] - positions[neighbor][0]
                    delta_y = positions[node][1] - positions[neighbor][1]
                    
                    distance = math.sqrt(delta_x**2 + delta_y**2)
                    if distance < 0.01:
                        distance = 0.01
                    
                    force = (distance * distance) / k
                    
                    displacement[node][0] -= (delta_x / distance) * force
                    displacement[node][1] -= (delta_y / distance) * force
                    displacement[neighbor][0] += (delta_x / distance) * force
                    displacement[neighbor][1] += (delta_y / distance) * force
            
            # Apply displacement with temperature limit
            for node in nodes:
                disp_x = displacement[node][0]
                disp_y = displacement[node][1]
                
                disp_length = math.sqrt(disp_x**2 + disp_y**2)
                if disp_length < 0.01:
                    disp_length = 0.01
                
                limited_disp = min(disp_length, temp) / disp_length
                
                positions[node][0] += disp_x * limited_disp
                positions[node][1] += disp_y * limited_disp
                
                # Boundaries
                node_w, node_h = node_dimensions[node]
                positions[node][0] = max(MARGIN_X, 
                                        min(positions[node][0], 
                                            viewport_width - MARGIN_X - node_w))
                positions[node][1] = max(MARGIN_Y, 
                                        min(positions[node][1], 
                                            viewport_height - MARGIN_Y - node_h))
            
            # Cool down
            temp = initial_temp * (1 - iteration / ITERATIONS)
        
        # Apply positions
        for node, (x, y) in positions.items():
            node.setPos(x, y)
        
        # Update connections
        for node in nodes:
            if hasattr(node, "connections"):
                for conn in node.connections:
                    conn.update_position()

    def _position_root_layers_sugiyama(self, layers, graph, reverse_graph):
        """Position root-level nodes using grid-based Sugiyama assignment.
        
        Uses adaptive layout that scales with viewport and content size.

        Args:
            layers: List of layers, each containing nodes
            graph: Forward adjacency graph (node -> list of successors)
            reverse_graph: Reverse adjacency graph (node -> list of predecessors)
        """
        # Get viewport dimensions to calculate available space
        viewport_rect = self.canvas.viewport().rect()
        viewport_width = viewport_rect.width()
        viewport_height = viewport_rect.height()
        
        # Adaptive margins based on viewport size
        MARGIN_RATIO_X = 0.08  # 8% margin on sides
        MARGIN_RATIO_Y = 0.1   # 10% margin top/bottom
        
        margin_x = viewport_width * MARGIN_RATIO_X
        margin_y = viewport_height * MARGIN_RATIO_Y
        
        START_X = margin_x
        START_Y = margin_y
        
        available_width = viewport_width - (2 * margin_x)
        available_height = viewport_height - (2 * margin_y)

        # Get element dimensions
        def get_element_size(node):
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                rect = node.rect()
                return rect.width(), rect.height()
            else:
                bbox = node.boundingRect()
                return bbox.width(), bbox.height()

        # Calculate dimensions
        node_dimensions = {}
        max_node_height = 0
        
        for layer in layers:
            for node in layer:
                width, height = get_element_size(node)
                node_dimensions[node] = (width, height)
                max_node_height = max(max_node_height, height)

        num_layers = len(layers)
        
        # Adaptive vertical spacing
        if num_layers > 1:
            total_nodes_height = num_layers * max_node_height
            space_for_gaps = available_height - total_nodes_height
            layer_spacing = space_for_gaps / (num_layers - 1) if space_for_gaps > 0 else max_node_height * 0.4
        else:
            layer_spacing = 0

        # Phase 1: Calculate layer Y positions with adaptive spacing
        layer_y_positions = []
        current_y = START_Y
        
        for layer_idx in range(num_layers):
            layer_y_positions.append(current_y)
            if layer_idx < num_layers - 1:
                current_y += max_node_height + layer_spacing

        # Phase 2: Initial X positioning with adaptive horizontal spacing
        positions = {}
        
        for layer_idx, layer in enumerate(layers):
            layer_y = layer_y_positions[layer_idx]
            
            if len(layer) == 0:
                continue
            
            # Calculate total width of nodes in this layer
            total_node_width = sum(node_dimensions[node][0] for node in layer)
            
            if len(layer) == 1:
                # Single node - center it
                node = layer[0]
                node_width = node_dimensions[node][0]
                x = START_X + (available_width - node_width) / 2
                positions[node] = (x, layer_y)
            else:
                # Multiple nodes - distribute edge to edge
                available_space = available_width - total_node_width
                
                if len(layer) > 1:
                    gap_size = available_space / (len(layer) - 1)
                else:
                    gap_size = 0
                
                current_x = START_X
                
                for node in layer:
                    positions[node] = (current_x, layer_y)
                    width = node_dimensions[node][0]
                    current_x += width + gap_size

        # Phase 3: Optimize with median positioning
        for iteration in range(5):
            for layer_idx, layer in enumerate(layers):
                if len(layer) <= 1:
                    continue

                for node in layer:
                    connected_positions = []
                    
                    # Predecessors
                    for pred in reverse_graph.get(node, []):
                        if pred in positions:
                            connected_positions.append(positions[pred][0])
                    
                    # Successors
                    for succ in graph.get(node, []):
                        if succ in positions:
                            connected_positions.append(positions[succ][0])
                    
                    if connected_positions:
                        connected_positions.sort()
                        median_x = connected_positions[len(connected_positions) // 2]
                        
                        current_x, current_y = positions[node]
                        new_x = current_x + (median_x - current_x) * 0.3
                        positions[node] = (new_x, current_y)

        # Calculate minimum spacing for collision detection
        # Use the smallest gap size as minimum spacing
        max_nodes_in_layer = max(len(layer) for layer in layers) if layers else 1
        min_spacing = available_width / (max_nodes_in_layer + 1) if max_nodes_in_layer > 1 else 100
        min_spacing = max(30, min_spacing)

        # Phase 4: Grid-based collision resolution
        positions = self._resolve_collisions_grid_based(positions, node_dimensions, min_spacing)

        # Phase 5: Grid-aware crossing minimization
        positions = self._minimize_crossings_grid_aware(
            positions, node_dimensions, layers, graph, reverse_graph, min_spacing
        )

        # Phase 6: Apply positions
        for node, (x, y) in positions.items():
            node.setPos(x, y)

        # Update connections
        all_nodes = [node for layer in layers for node in layer]
        for node in all_nodes:
                layer_nodes_with_pos = [(positions[node][0], node) for node in layer]
                layer_nodes_with_pos.sort()

        # Update connections
        all_nodes = [node for layer in layers for node in layer]
        for node in all_nodes:
            for conn in node.connections if hasattr(node, "connections") else []:
                conn.update_position()

    def _load_states_from_xml(self, parent_elem, parent_container):
        """Recursively load states from XML, handling nested containers."""
        for elem in parent_elem:
            if elem.tag == "State" or (
                elem.tag == "StateMachine" and elem.get("file_name")
            ):
                state_name = elem.get("name")
                state_type = elem.get("type")
                remappings = self._load_remappings(elem)

                plugin_info = None
                if state_type == "py":
                    module = elem.get("module")
                    class_name = elem.get("class")
                    for plugin in self.plugin_manager.python_plugins:
                        if plugin.module == module and plugin.class_name == class_name:
                            plugin_info = plugin
                            break
                elif state_type == "cpp":
                    class_name = elem.get("class")
                    for plugin in self.plugin_manager.cpp_plugins:
                        if plugin.class_name == class_name:
                            plugin_info = plugin
                            break
                elif state_type == "xml":
                    file_name = elem.get("file_name")
                    for plugin in self.plugin_manager.xml_files:
                        if plugin.file_name == file_name:
                            plugin_info = plugin
                            break

                if plugin_info:
                    node = StateNode(state_name, plugin_info, 0, 0, remappings)
                    if parent_container is None:
                        self.canvas.scene.addItem(node)
                        self.state_nodes[state_name] = node
                    else:
                        parent_container.add_child_state(node)
                        full_name = f"{parent_container.name}.{state_name}"
                        self.state_nodes[full_name] = node

            elif elem.tag == "StateMachine" and not elem.get("file_name"):
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                init_state = elem.get("start_state", "")
                remappings = self._load_remappings(elem)
                outcomes = outcomes_str.split() if outcomes_str else []

                node = ContainerStateNode(
                    state_name, 0, 0, False, remappings, outcomes, init_state
                )
                if parent_container is None:
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                else:
                    parent_container.add_child_state(node)
                    full_name = f"{parent_container.name}.{state_name}"
                    self.state_nodes[full_name] = node

                for outcome in outcomes:
                    outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    node.add_final_outcome(outcome_node)

                self._load_states_from_xml(elem, node)

            elif elem.tag == "Concurrence":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                default_outcome = elem.get("default_outcome", None)
                remappings = self._load_remappings(elem)
                outcomes = outcomes_str.split() if outcomes_str else []

                node = ContainerStateNode(
                    state_name, 0, 0, True, remappings, outcomes, None, default_outcome
                )
                if parent_container is None:
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                else:
                    parent_container.add_child_state(node)
                    full_name = f"{parent_container.name}.{state_name}"
                    self.state_nodes[full_name] = node

                for outcome in outcomes:
                    outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    node.add_final_outcome(outcome_node)

                self._load_states_from_xml(elem, node)

    def _load_remappings(self, elem):
        """Helper method to load remappings from an XML element."""
        remappings = {}
        for remap in elem.findall("Remap"):
            from_key = remap.get("old", "")
            to_key = remap.get("new", "")
            if from_key and to_key:
                remappings[from_key] = to_key
        return remappings

    def _load_transitions_from_xml(self, parent_elem, parent_container):
        """Recursively load transitions from XML."""
        for elem in parent_elem:
            if elem.tag in ["State", "StateMachine", "Concurrence"]:
                state_name = elem.get("name")

                # Find the from_node
                if parent_container is None:
                    from_node = self.state_nodes.get(state_name)
                else:
                    full_name = f"{parent_container.name}.{state_name}"
                    from_node = self.state_nodes.get(full_name)

                if from_node:
                    # For containers, collect final outcome names to distinguish
                    # transitions from child states vs transitions from final outcomes
                    final_outcome_names = set()
                    if elem.tag in ["StateMachine", "Concurrence"] and hasattr(
                        from_node, "final_outcomes"
                    ):
                        final_outcome_names = set(from_node.final_outcomes.keys())

                    for transition in elem.findall("Transition"):
                        outcome = transition.get("from")
                        to_name = transition.get("to")

                        # Determine if this transition is from a final outcome or from a child state
                        is_from_final_outcome = outcome in final_outcome_names

                        if is_from_final_outcome:
                            # Transition from a final outcome inside the container
                            from_outcome = from_node.final_outcomes[outcome]

                            # Find the target node
                            to_node = None

                            # Check if target is a root-level node or final outcome
                            if parent_container is None:
                                to_node = self.state_nodes.get(to_name)
                                if not to_node:
                                    to_node = self.final_outcomes.get(to_name)
                            # Check if target is in parent container's final outcomes
                            elif to_name in parent_container.final_outcomes:
                                to_node = parent_container.final_outcomes[to_name]
                            # Check if target is a sibling state
                            else:
                                full_to_name = f"{parent_container.name}.{to_name}"
                                to_node = self.state_nodes.get(full_to_name)

                            if to_node:
                                connection = ConnectionLine(
                                    from_outcome, to_node, outcome
                                )
                                self.canvas.scene.addItem(connection)
                                self.canvas.scene.addItem(connection.arrow_head)
                                self.canvas.scene.addItem(connection.label_bg)
                                self.canvas.scene.addItem(connection.label)
                                self.connections.append(connection)
                        else:
                            # Transition from a child state inside the container
                            # Find target node - could be in same container or a final outcome
                            to_node = None

                            if parent_container is None:
                                # Root level - check root states and final outcomes
                                to_node = self.state_nodes.get(to_name)
                                if not to_node:
                                    to_node = self.final_outcomes.get(to_name)
                            else:
                                # Inside a container - check sibling states and final outcomes
                                # First check if it's a final outcome of this container
                                if to_name in parent_container.final_outcomes:
                                    to_node = parent_container.final_outcomes[to_name]
                                else:
                                    # Check if it's a sibling state
                                    full_to_name = f"{parent_container.name}.{to_name}"
                                    to_node = self.state_nodes.get(full_to_name)

                            if to_node:
                                connection = ConnectionLine(from_node, to_node, outcome)
                                self.canvas.scene.addItem(connection)
                                self.canvas.scene.addItem(connection.arrow_head)
                                self.canvas.scene.addItem(connection.label_bg)
                                self.canvas.scene.addItem(connection.label)
                                self.connections.append(connection)

                # If this is a container, recursively load transitions from children
                if elem.tag in ["StateMachine", "Concurrence"]:
                    if parent_container is None:
                        container = self.state_nodes.get(state_name)
                    else:
                        full_name = f"{parent_container.name}.{state_name}"
                        container = self.state_nodes.get(full_name)

                    # Recursively load transitions from children
                    if container:
                        self._load_transitions_from_xml(elem, container)
