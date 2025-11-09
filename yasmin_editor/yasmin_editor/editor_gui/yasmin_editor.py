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
import xml.etree.ElementTree as ET
from xml.dom import minidom
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
from PyQt5.QtCore import Qt, QPointF, QRectF

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
    """Main editor window for YASMIN state machines."""

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
        new_action.triggered.connect(self.new_state_machine)
        toolbar.addAction(new_action)

        open_action = QAction("Open", self)
        open_action.triggered.connect(self.open_state_machine)
        toolbar.addAction(open_action)

        save_action = QAction("Save", self)
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
            filename = os.path.basename(xml_plugin.file_path)
            display_name = (
                f"{xml_plugin.package_name}::{filename}"
                if xml_plugin.package_name
                else filename
            )
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
        # Store current selection based on self.start_state (not combo text)
        current = self.start_state
        self.start_state_combo.clear()
        self.start_state_combo.addItem("(None)")

        for state_name in self.state_nodes.keys():
            self.start_state_combo.addItem(state_name)

        # Restore selection based on self.start_state
        if current:
            index = self.start_state_combo.findText(current)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)
            else:
                # If the initial state no longer exists, reset to None
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
        """Get a free position in the center of the current view or nearby."""
        # Get the center of the current viewport in scene coordinates
        viewport_rect = self.canvas.viewport().rect()
        center_in_view = viewport_rect.center()
        center_in_scene = self.canvas.mapToScene(center_in_view)

        # Start from the center
        test_pos = center_in_scene
        search_radius = 50
        max_attempts = 20

        for attempt in range(max_attempts):
            # Check if this position is free (no overlap with existing nodes)
            is_free = True
            test_rect = QRectF(test_pos.x() - 60, test_pos.y() - 40, 120, 80)

            for node in self.state_nodes.values():
                # Only check root-level nodes
                if hasattr(node, "parent_container") and node.parent_container:
                    continue

                node_rect = QRectF(
                    node.pos().x() + node.boundingRect().left(),
                    node.pos().y() + node.boundingRect().top(),
                    node.boundingRect().width(),
                    node.boundingRect().height(),
                )

                if test_rect.intersects(node_rect):
                    is_free = False
                    break

            if is_free:
                return test_pos

            # Spiral search pattern
            angle = attempt * 0.5
            radius = search_radius * (1 + attempt * 0.3)

            test_pos = QPointF(
                center_in_scene.x() + radius * math.cos(angle),
                center_in_scene.y() + radius * math.sin(angle),
            )

        # If no free position found, return the center anyway
        return center_in_scene

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

        # Get free position in viewport center or closest free space
        pos = self.get_free_position()

        # Create node
        # XML state machines are regular StateNode (not containers)
        # Only user-created State Machines and Concurrences are containers
        if is_state_machine or is_concurrence:
            # User-created container (State Machine or Concurrence)
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
            # Regular state with plugin
            node = StateNode(
                name,
                plugin_info,
                pos.x(),
                pos.y(),
                remappings,
            )

        self.canvas.scene.addItem(node)
        self.state_nodes[name] = node

        # Update initial state combo
        self.update_start_state_combo()

        # If this is the first state and no initial state is set, set it as initial
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

        # Store old name for renaming
        old_name = state_node.name

        # Use appropriate dialog based on state type
        if isinstance(state_node, ContainerStateNode):
            if state_node.is_state_machine:
                # Use StateMachineDialog for State Machine containers
                # Get list of child state names
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

                        # Handle name change
                        if name != old_name:
                            # Check if new name already exists
                            if name in self.state_nodes:
                                QMessageBox.warning(
                                    self, "Error", f"State '{name}' already exists!"
                                )
                                return

                            # Update root initial state if this was the initial state
                            if self.start_state == old_name:
                                self.start_state = name

                            # Update state_nodes dictionary
                            del self.state_nodes[old_name]
                            self.state_nodes[name] = state_node

                            # Update node's name
                            state_node.name = name

                            # Update visual title
                            state_node.title.setPlainText(f"STATE MACHINE: {name}")
                            title_rect = state_node.title.boundingRect()
                            state_node.title.setPos(-title_rect.width() / 2, -75)

                            # Update initial state combo (will restore the selection)
                            self.update_start_state_combo()

                        # Update remappings
                        state_node.remappings = remappings

                        # Update initial state
                        if start_state:
                            state_node.start_state = start_state
                            state_node.update_start_state_label()

                        self.statusBar().showMessage(
                            f"Updated state machine: {name}", 2000
                        )
            elif state_node.is_concurrence:
                # Use ConcurrenceDialog for Concurrence containers
                # Get list of final outcome names
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

                        # Handle name change
                        if name != old_name:
                            # Check if new name already exists
                            if name in self.state_nodes:
                                QMessageBox.warning(
                                    self, "Error", f"State '{name}' already exists!"
                                )
                                return

                            # Update root initial state if this was the initial state
                            if self.start_state == old_name:
                                self.start_state = name

                            # Update state_nodes dictionary
                            del self.state_nodes[old_name]
                            self.state_nodes[name] = state_node

                            # Update node's name
                            state_node.name = name

                            # Update visual title
                            state_node.title.setPlainText(f"CONCURRENCE: {name}")
                            title_rect = state_node.title.boundingRect()
                            state_node.title.setPos(-title_rect.width() / 2, -75)

                            # Update initial state combo (will restore the selection)
                            self.update_start_state_combo()

                        # Update remappings
                        state_node.remappings = remappings

                        # Update default outcome
                        if default_outcome:
                            state_node.default_outcome = default_outcome
                            state_node.update_default_outcome_label()

                        self.statusBar().showMessage(f"Updated concurrence: {name}", 2000)
        else:
            # Use StatePropertiesDialog for regular states
            # In edit mode, only pass the selected state's plugin, not all available plugins
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
                if result[0]:  # Check if name is not None (validation passed)
                    (
                        name,
                        plugin,
                        outcomes,
                        remappings,
                    ) = result

                    # Handle name change
                    if name != old_name:
                        # Check if new name already exists
                        if name in self.state_nodes:
                            QMessageBox.warning(
                                self, "Error", f"State '{name}' already exists!"
                            )
                            return

                        # Update root initial state if this was the initial state
                        if self.start_state == old_name:
                            self.start_state = name

                        # Update state_nodes dictionary
                        del self.state_nodes[old_name]
                        self.state_nodes[name] = state_node

                        # Update node's name
                        state_node.name = name

                        # Update visual text label
                        state_node.text.setPlainText(name)
                        text_rect = state_node.text.boundingRect()
                        state_node.text.setPos(
                            -text_rect.width() / 2, -text_rect.height() / 2
                        )

                        # Update initial state combo (will restore the selection)
                        self.update_start_state_combo()

                    # Update remappings
                    state_node.remappings = remappings

                    self.statusBar().showMessage(f"Updated state: {name}", 2000)

    def add_state_to_container(self):
        """Add a child state to the selected container (SM or Concurrence)."""
        # Get selected container
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                # Only user-created containers (not XML-based)
                if not hasattr(item, "xml_file") or item.xml_file is None:
                    container = item
                    break

        if not container:
            QMessageBox.warning(
                self,
                "Error",
                "Please select a user-created Container State Machine or Concurrence!",
            )
            return

        # Create dialog for adding a state
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
            if result[0]:  # Validation passed
                (
                    name,
                    plugin,
                    outcomes,
                    remappings,
                ) = result

                # Check if child state already exists in container
                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                # Create the child state node
                child_node = StateNode(name, plugin, 0, 0, remappings)

                # Add to container (this will set parent and position)
                container.add_child_state(child_node)

                # Note: Don't add to scene separately - it's added via setParentItem in add_child_state

                # Also add to global state nodes dict for transitions
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_node

                self.statusBar().showMessage(
                    f"Added state '{name}' to container '{container.name}'", 2000
                )

                # Auto-assign initial state if this is the first state in a State Machine
                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def add_state_machine_to_container(self):
        """Add a State Machine to the selected container."""
        # Get selected container
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                # Only user-created containers (not XML-based)
                if not hasattr(item, "xml_file") or item.xml_file is None:
                    container = item
                    break

        if not container:
            QMessageBox.warning(
                self,
                "Error",
                "Please select a user-created Container!",
            )
            return

        dialog = StateMachineDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:
                name, outcomes, start_state, remappings = result

                # Check if child already exists
                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                # Create nested state machine
                child_sm = ContainerStateNode(
                    name,
                    0,
                    0,
                    False,  # is_concurrence
                    remappings,
                    outcomes,
                    start_state,
                    None,
                )

                # Add to container
                container.add_child_state(child_sm)

                # Add to global dict
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_sm

                self.statusBar().showMessage(
                    f"Added State Machine '{name}' to container '{container.name}'", 2000
                )

                # Auto-assign initial state if this is the first state in a State Machine
                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def add_concurrence_to_container(self):
        """Add a Concurrence to the selected container."""
        # Get selected container
        selected_items = self.canvas.scene.selectedItems()
        container = None

        for item in selected_items:
            if isinstance(item, ContainerStateNode):
                # Only user-created containers (not XML-based)
                if not hasattr(item, "xml_file") or item.xml_file is None:
                    container = item
                    break

        if not container:
            QMessageBox.warning(
                self,
                "Error",
                "Please select a user-created Container!",
            )
            return

        dialog = ConcurrenceDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:
                name, outcomes, default_outcome, remappings = result

                # Check if child already exists
                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                # Create nested concurrence
                child_cc = ContainerStateNode(
                    name,
                    0,
                    0,
                    True,  # is_concurrence
                    remappings,
                    outcomes,
                    None,
                    default_outcome,
                )

                # Add to container
                container.add_child_state(child_cc)

                # Add to global dict
                full_name = f"{container.name}.{name}"
                self.state_nodes[full_name] = child_cc

                self.statusBar().showMessage(
                    f"Added Concurrence '{name}' to container '{container.name}'", 2000
                )

                # Auto-assign initial state if this is the first state in a State Machine
                if container.is_state_machine and len(container.child_states) == 1:
                    container.start_state = name
                    container.update_start_state_label()

    def create_connection_from_drag(self, from_node, to_node):
        """Create a connection when user drags from one node to another."""
        # Containers (SM/Concurrence) cannot have external transitions
        # They use final outcomes inside the container instead
        if isinstance(from_node, ContainerStateNode):
            QMessageBox.warning(
                self,
                "Not Allowed",
                "Container State Machines and Concurrence states cannot have external transitions.\n"
                "Use Final Outcomes inside the container to define exit points.",
            )
            return

        # Check if from_node has outcomes
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

        # Check if this state is inside a Concurrence
        is_in_concurrence = False
        parent_concurrence = None
        if hasattr(from_node, "parent_container") and from_node.parent_container:
            if isinstance(from_node.parent_container, ContainerStateNode):
                is_in_concurrence = from_node.parent_container.is_concurrence
                if is_in_concurrence:
                    parent_concurrence = from_node.parent_container

        # If state is inside a Concurrence, it can only transition to final outcomes inside the same Concurrence
        if is_in_concurrence:
            # Check if target is a final outcome inside the same concurrence
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

        # Filter out already used outcomes (only if NOT in a Concurrence)
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
                    self,
                    "Error",
                    "All outcomes from this state are already used!",
                )
                return
        else:
            # In a Concurrence, allow reusing outcomes
            available_outcomes = outcomes_list

        # If the from_node has only one available outcome, use it directly
        if len(available_outcomes) == 1:
            outcome = available_outcomes[0]
            self.create_connection(from_node, to_node, outcome)
        else:
            # Ask user to select outcome
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
        # Check if this state is inside a Concurrence
        is_in_concurrence = False
        if hasattr(from_node, "parent_container") and from_node.parent_container:
            if isinstance(from_node.parent_container, ContainerStateNode):
                is_in_concurrence = from_node.parent_container.is_concurrence

        # Check if outcome is already used (only if NOT in a Concurrence)
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

        # Update all parallel connections to recalculate offsets
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
        # Get selected state
        selected_items = self.canvas.scene.selectedItems()
        from_state = None

        for item in selected_items:
            if isinstance(item, StateNode):  # Only regular states, not containers
                from_state = item
                break

        if not from_state:
            QMessageBox.warning(
                self, "Error", "Please select a regular state (not a container) first!"
            )
            return

        # Check if state has outcomes
        has_outcomes = False
        if hasattr(from_state, "plugin_info") and from_state.plugin_info:
            has_outcomes = True

        if not has_outcomes:
            QMessageBox.warning(
                self,
                "Error",
                "Selected state has no outcomes!",
            )
            return

        # Check if state is inside a Concurrence
        is_in_concurrence = False
        parent_concurrence = None
        if hasattr(from_state, "parent_container") and from_state.parent_container:
            if isinstance(from_state.parent_container, ContainerStateNode):
                is_in_concurrence = from_state.parent_container.is_concurrence
                if is_in_concurrence:
                    parent_concurrence = from_state.parent_container

        # Get available targets
        available_targets = []

        if is_in_concurrence:
            # If state is inside a Concurrence, only allow transitions to final outcomes inside the same Concurrence
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
            # For states not in a Concurrence, exclude containers as they can't be transition targets
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
            if outcome:  # Make sure outcome is selected
                self.create_connection(from_state, target, outcome)

    def add_final_outcome(self):
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        if ok and outcome_name:
            # Check if a container is selected to add final outcome inside it
            selected_items = self.canvas.scene.selectedItems()
            selected_container = None

            for item in selected_items:
                if isinstance(item, ContainerStateNode):
                    # Only user-created containers (not XML-based)
                    if not hasattr(item, "xml_file") or item.xml_file is None:
                        selected_container = item
                        break

            if selected_container:
                # Add final outcome to the container
                if outcome_name in selected_container.final_outcomes:
                    QMessageBox.warning(
                        self,
                        "Error",
                        f"Final outcome '{outcome_name}' already exists in this container!",
                    )
                    return

                # Create final outcome node inside container
                # Position on the right side of the container
                rect = selected_container.rect()
                x = rect.right() - 100  # Near right edge
                y = rect.top() + 70 + len(selected_container.final_outcomes) * 80

                node = FinalOutcomeNode(outcome_name, x, y, inside_container=True)
                node.parent_container = selected_container
                node.setParentItem(selected_container)
                selected_container.final_outcomes[outcome_name] = node
                selected_container.auto_resize_for_children()

                # Auto-assign default outcome if this is the first outcome in a Concurrence
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
                # Add final outcome to root level
                if outcome_name in self.final_outcomes:
                    QMessageBox.warning(
                        self, "Error", f"Final outcome '{outcome_name}' already exists!"
                    )
                    return

                # Create final outcome node (positioned on the right side)
                # Position based on existing final outcomes count
                x = 600  # Fixed x position on the right
                y = len(self.final_outcomes) * 150
                node = FinalOutcomeNode(outcome_name, x, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome_name] = node
                self.statusBar().showMessage(f"Added final outcome: {outcome_name}", 2000)

    def delete_selected(self):
        selected_items = self.canvas.scene.selectedItems()

        for item in selected_items:
            if isinstance(item, (StateNode, ContainerStateNode)):
                # Remove all connections - properly clean up both ends
                for connection in item.connections[:]:
                    # Remove from the other node's connection list
                    if connection.from_node == item:
                        connection.to_node.remove_connection(connection)
                    else:
                        connection.from_node.remove_connection(connection)

                    # Remove from scene
                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                # Handle nested states/containers
                if hasattr(item, "parent_container") and item.parent_container:
                    # This is a child state/container inside another container
                    parent = item.parent_container

                    # Remove from parent's child_states dict
                    if item.name in parent.child_states:
                        del parent.child_states[item.name]

                    # Remove from global state_nodes with full path
                    full_name = f"{parent.name}.{item.name}"
                    if full_name in self.state_nodes:
                        del self.state_nodes[full_name]

                    # Update parent container size
                    parent.auto_resize_for_children()

                    # Remove from scene
                    self.canvas.scene.removeItem(item)
                    self.statusBar().showMessage(
                        f"Deleted nested state: {item.name}", 2000
                    )
                else:
                    # Root-level state/container
                    # Remove node
                    self.canvas.scene.removeItem(item)
                    if item.name in self.state_nodes:
                        del self.state_nodes[item.name]
                    self.update_start_state_combo()  # Update combo after deletion
                    self.statusBar().showMessage(f"Deleted state: {item.name}", 2000)

            elif isinstance(item, FinalOutcomeNode):
                # Remove all connections - properly clean up both ends
                for connection in item.connections[:]:
                    # Remove from the other node's connection list
                    if connection.from_node == item:
                        connection.to_node.remove_connection(connection)
                    else:
                        connection.from_node.remove_connection(connection)

                    # Remove from scene
                    self.canvas.scene.removeItem(connection)
                    self.canvas.scene.removeItem(connection.arrow_head)
                    self.canvas.scene.removeItem(connection.label_bg)
                    self.canvas.scene.removeItem(connection.label)
                    if connection in self.connections:
                        self.connections.remove(connection)

                # Remove node - check if it's in a container or at root level
                if item.parent_container:
                    # Remove from container
                    if item.name in item.parent_container.final_outcomes:
                        del item.parent_container.final_outcomes[item.name]
                    item.parent_container.auto_resize_for_children()
                    self.canvas.scene.removeItem(item)
                else:
                    # Remove from root level
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
        root.set("outcomes", " ".join(self.final_outcomes.keys()))
        if ok and sm_name:
            root.set("name", sm_name)

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

        # Pretty print XML
        xml_str = ET.tostring(root, encoding="unicode")
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")

        # Remove extra blank lines
        pretty_xml = "\n".join([line for line in pretty_xml.split("\n") if line.strip()])

        # Check if file_path ends with .xml
        if not file_path.lower().endswith(".xml"):
            file_path += ".xml"

        # Write to file
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(pretty_xml)

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
                state_elem = ET.SubElement(parent_elem, "State")
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
                        # XML-based state machine reference
                        state_elem.set("type", "xml")
                        if hasattr(state_node, "xml_file") and state_node.xml_file:
                            state_elem.set("file", state_node.xml_file)

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
        self.new_state_machine()

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

        # Reorganize all containers to properly layout their children
        # This must be done BEFORE repositioning root elements so containers have their final sizes
        self._reorganize_all_containers()

        # Force scene update to ensure all geometries are calculated
        self.canvas.scene.update()
        QApplication.processEvents()  # Process any pending events

        # Now reposition root-level elements based on actual sizes after auto-resize
        self._reposition_root_elements_after_resize()

        # Force another scene update after repositioning and ensure geometry is calculated
        self.canvas.scene.update()
        QApplication.processEvents()  # Process any pending events to update geometries

        # Get final outcomes and position them properly AFTER states are resized and repositioned
        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()

            # Find the rightmost position of all root-level nodes (now with correct sizes)
            max_x = 0
            max_y = 0
            for state_node in self.state_nodes.values():
                # Only check root-level nodes
                if (
                    not hasattr(state_node, "parent_container")
                    or state_node.parent_container is None
                ):
                    # Ensure geometry is up to date before measuring
                    if isinstance(state_node, ContainerStateNode):
                        state_node.prepareGeometryChange()
                        rect = state_node.rect()
                        # Use actual width from rect
                        node_right = state_node.pos().x() + rect.width()
                        node_bottom = state_node.pos().y() + rect.height()
                    else:
                        bbox = state_node.boundingRect()
                        node_right = state_node.pos().x() + bbox.width()
                        node_bottom = state_node.pos().y() + bbox.height()
                    max_x = max(max_x, node_right)
                    max_y = max(max_y, node_bottom)

            # Position final outcomes to the right of all states with generous padding
            # Extra spacing to prevent connection lines from overlapping outcomes
            outcome_x = (
                max_x + 300 if max_x > 0 else 900
            )  # Increased for better clearance
            outcome_start_y = 140  # Increased start position
            outcome_spacing = (
                180  # Increased vertical spacing to prevent connection overlap
            )

            for i, outcome in enumerate(outcomes):
                y = outcome_start_y + i * outcome_spacing
                node = FinalOutcomeNode(outcome, outcome_x, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome] = node

        # Update initial state combo
        self.update_start_state_combo()

        # Set initial state selection
        if start_state:
            index = self.start_state_combo.findText(start_state)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)

        # Load transitions recursively
        self._load_transitions_from_xml(root, None)

        # Final scene update to ensure all connections are drawn correctly
        self.canvas.scene.update()
        QApplication.processEvents()

    def _reorganize_all_containers(self):
        """Reorganize all containers and their children after loading.
        Uses graph-based layout algorithms for optimal placement.
        """
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

        # Process each container with graph-based layout
        for container in all_containers:
            self._layout_container_with_graph(container)

        # Final pass to ensure all sizes are correct
        for container in all_containers:
            container.prepareGeometryChange()
            container.auto_resize_for_children()

        # One more geometry update to finalize everything
        self.canvas.scene.update()
        QApplication.processEvents()

    def _layout_container_with_graph(self, container: ContainerStateNode):
        """Layout children within a container using hierarchical graph layout.
        Uses Sugiyama-style layering with force-directed positioning.
        """
        if not container.child_states and not container.final_outcomes:
            return

        # Build a directed graph of states and their transitions
        nodes = list(container.child_states.values())
        final_outcomes = list(container.final_outcomes.values())

        if not nodes:
            # Only final outcomes, position them vertically
            rect = container.rect()
            y_start = rect.top() + 140
            for i, outcome in enumerate(final_outcomes):
                outcome.setPos(rect.left() + 120, y_start + i * 120)
            container.auto_resize_for_children()
            return

        # Build adjacency list for the graph
        graph = {node: [] for node in nodes}
        for node in nodes:
            for conn in node.connections:
                if conn.from_node == node and conn.to_node in nodes:
                    graph[node].append(conn.to_node)

        # Perform topological layering (assign nodes to layers)
        layers = self._compute_layers(nodes, graph, container.start_state)

        # Position nodes using hierarchical layout
        self._position_layers(container, layers, final_outcomes)

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
        """Position nodes in layers with proper spacing to avoid overlaps.
        Uses a force-directed approach for final positioning.
        """
        rect = container.rect()

        # Configuration
        LAYER_SPACING = 280  # Horizontal spacing between layers
        NODE_SPACING = 150  # Vertical spacing between nodes in same layer
        START_X = rect.left() + 140  # Left padding
        START_Y = rect.top() + 160  # Top padding (below header)

        # Build adjacency info for better positioning
        graph = {node: [] for layer in layers for node in layer}
        for layer in layers:
            for node in layer:
                for conn in node.connections:
                    if conn.from_node == node:
                        graph[node].append(conn.to_node)

        # First pass: position nodes in layers
        layer_positions = {}  # Map node -> (x, y)

        for layer_idx, layer in enumerate(layers):
            x = START_X + layer_idx * LAYER_SPACING

            # Calculate total height needed for this layer
            total_height = 0
            node_heights = []
            for node in layer:
                if isinstance(node, ContainerStateNode):
                    node.prepareGeometryChange()
                    height = node.rect().height()
                else:
                    height = node.boundingRect().height()
                node_heights.append(height)
                total_height += height

            # Add spacing between nodes
            total_height += NODE_SPACING * (len(layer) - 1) if len(layer) > 1 else 0

            # Apply barycenter heuristic for vertical positioning
            # This minimizes edge crossings by positioning nodes near their neighbors
            if layer_idx > 0:
                # Position based on average position of predecessors
                node_barycenters = []
                for node in layer:
                    # Find predecessors in previous layer
                    predecessors = []
                    for prev_node in layers[layer_idx - 1]:
                        if node in graph.get(prev_node, []):
                            predecessors.append(prev_node)

                    if predecessors:
                        # Calculate average Y position of predecessors
                        avg_y = sum(layer_positions[p][1] for p in predecessors) / len(
                            predecessors
                        )
                        node_barycenters.append((node, avg_y))
                    else:
                        # No predecessors, use default position
                        node_barycenters.append((node, START_Y))

                # Sort by barycenter to minimize crossings
                node_barycenters.sort(key=lambda x: x[1])
                layer = [n for n, _ in node_barycenters]

            # Position nodes vertically
            current_y = START_Y

            for node, height in zip(layer, node_heights):
                layer_positions[node] = (x, current_y)
                node.setPos(x, current_y)
                current_y += height + NODE_SPACING

        # Apply force-directed refinement to reduce edge crossings
        self._refine_positions_force_directed(layers, layer_positions, graph)

        # Position final outcomes to the right of all layers
        if final_outcomes and layers:
            max_x = START_X + len(layers) * LAYER_SPACING
            outcome_x = max_x + 200  # Extra spacing for outcomes

            # Position outcomes based on their incoming connections
            outcome_positions = {}
            for outcome in final_outcomes:
                # Find all nodes that connect to this outcome
                incoming_nodes = []
                for layer in layers:
                    for node in layer:
                        for conn in node.connections:
                            if conn.to_node == outcome:
                                incoming_nodes.append(node)

                if incoming_nodes:
                    # Position near average of incoming nodes
                    avg_y = sum(
                        layer_positions[n][1]
                        for n in incoming_nodes
                        if n in layer_positions
                    ) / len(incoming_nodes)
                    outcome_positions[outcome] = avg_y
                else:
                    outcome_positions[outcome] = START_Y

            # Sort outcomes by their target position
            sorted_outcomes = sorted(final_outcomes, key=lambda o: outcome_positions[o])

            # Position with minimum spacing
            current_y = START_Y
            for outcome in sorted_outcomes:
                outcome.setPos(outcome_x, current_y)
                current_y += 180

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
        """Reposition root-level elements using graph-based hierarchical layout.
        This ensures optimal spacing and prevents overlapping.
        """
        # Get all root-level nodes
        root_nodes = []
        for node in self.state_nodes.values():
            if not hasattr(node, "parent_container") or node.parent_container is None:
                root_nodes.append(node)

        if not root_nodes:
            return

        # Build graph of root-level transitions
        graph = {node: [] for node in root_nodes}
        for node in root_nodes:
            for conn in node.connections:
                if conn.from_node == node and conn.to_node in root_nodes:
                    graph[node].append(conn.to_node)

        # Compute hierarchical layers
        layers = self._compute_layers(root_nodes, graph, self.start_state)

        # Position using hierarchical layout with generous spacing
        self._position_root_layers(layers)

    def _position_root_layers(self, layers):
        """Position root-level nodes in layers with optimal spacing."""
        # Configuration for root level (more generous spacing)
        LAYER_SPACING = 450  # Wide horizontal spacing between layers
        NODE_SPACING = 200  # Vertical spacing between nodes in same layer
        START_X = 150  # Left padding
        START_Y = 150  # Top padding

        # Helper function to get element dimensions
        def get_element_size(node):
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                rect = node.rect()
                return rect.width(), rect.height()
            else:
                bbox = node.boundingRect()
                return bbox.width(), bbox.height()

        # Calculate maximum width for each layer
        layer_widths = []
        for layer in layers:
            max_width = 0
            for node in layer:
                width, _ = get_element_size(node)
                max_width = max(max_width, width)
            layer_widths.append(max_width)

        # Position nodes layer by layer
        current_x = START_X

        for layer_idx, layer in enumerate(layers):
            # Calculate positions for nodes in this layer
            layer_node_heights = []
            for node in layer:
                _, height = get_element_size(node)
                layer_node_heights.append(height)

            # Calculate total height needed
            total_height = sum(layer_node_heights)
            if len(layer) > 1:
                total_height += NODE_SPACING * (len(layer) - 1)

            # Start from top
            current_y = START_Y

            # Position each node in the layer
            for node, height in zip(layer, layer_node_heights):
                node.setPos(current_x, current_y)
                current_y += height + NODE_SPACING

            # Move to next layer
            current_x += layer_widths[layer_idx] + LAYER_SPACING

        # Update all connections after repositioning
        for layer in layers:
            for node in layer:
                for conn in node.connections:
                    conn.update_position()

    def _load_states_from_xml(self, parent_elem, parent_container):
        """Recursively load states from XML, handling nested containers."""
        # Counter for positioning root-level states
        root_state_index = 0

        # For root-level elements, use temporary positioning (0,0)
        # They will be properly positioned after all sizes are calculated

        for elem in parent_elem:
            if elem.tag == "State":
                state_name = elem.get("name")
                state_type = elem.get("type")

                # Load remappings
                remappings = self._load_remappings(elem)

                # Find plugin
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

                if plugin_info:
                    if parent_container is None:
                        # Root level state - use temporary position, will be repositioned later
                        node = StateNode(state_name, plugin_info, 0, 0, remappings)
                        node.grid_index = (
                            root_state_index  # Track grid position for repositioning
                        )
                        self.canvas.scene.addItem(node)
                        self.state_nodes[state_name] = node
                        root_state_index += 1
                    else:
                        # Nested state inside a container - position will be auto-arranged
                        node = StateNode(state_name, plugin_info, 0, 0, remappings)
                        parent_container.add_child_state(node)
                        # Add to global state nodes with full path
                        full_name = f"{parent_container.name}.{state_name}"
                        self.state_nodes[full_name] = node

            elif elem.tag == "StateMachine":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                init_state = elem.get("start_state", "")

                # Load remappings
                remappings = self._load_remappings(elem)

                # Parse outcomes
                outcomes = outcomes_str.split() if outcomes_str else []

                if parent_container is None:
                    # Root level state machine - use temporary position, will be repositioned later
                    node = ContainerStateNode(
                        state_name, 0, 0, False, remappings, outcomes, init_state
                    )
                    node.grid_index = (
                        root_state_index  # Track grid position for repositioning
                    )
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                    root_state_index += 1
                else:
                    # Nested state machine inside a container
                    node = ContainerStateNode(
                        state_name, 0, 0, False, remappings, outcomes, init_state
                    )
                    parent_container.add_child_state(node)
                    # Add to global state nodes with full path
                    full_name = f"{parent_container.name}.{state_name}"
                    self.state_nodes[full_name] = node

                # Load final outcomes for this state machine
                for outcome in outcomes:
                    outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    node.add_final_outcome(outcome_node)

                # Recursively load children of this state machine
                self._load_states_from_xml(elem, node)

            elif elem.tag == "Concurrence":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                default_outcome = elem.get("default_outcome", None)

                # Load remappings
                remappings = self._load_remappings(elem)

                # Parse outcomes
                outcomes = outcomes_str.split() if outcomes_str else []

                if parent_container is None:
                    # Root level concurrence - use temporary position, will be repositioned later
                    node = ContainerStateNode(
                        state_name,
                        0,
                        0,
                        True,
                        remappings,
                        outcomes,
                        None,
                        default_outcome,
                    )
                    node.grid_index = (
                        root_state_index  # Track grid position for repositioning
                    )
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                    root_state_index += 1
                else:
                    # Nested concurrence inside a container
                    node = ContainerStateNode(
                        state_name,
                        0,
                        0,
                        True,
                        remappings,
                        outcomes,
                        None,
                        default_outcome,
                    )
                    parent_container.add_child_state(node)
                    # Add to global state nodes with full path
                    full_name = f"{parent_container.name}.{state_name}"
                    self.state_nodes[full_name] = node

                # Load final outcomes for this concurrence
                for outcome in outcomes:
                    outcome_node = FinalOutcomeNode(outcome, 0, 0, inside_container=True)
                    node.add_final_outcome(outcome_node)

                # Recursively load children of this concurrence
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
