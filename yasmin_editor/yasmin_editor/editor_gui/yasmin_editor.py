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
        self.next_state_position = QPointF(0, 0)
        self.root_sm_name = ""  # Root state machine name
        self.initial_state = None  # Initial state for root SM

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
        import os

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

        add_transition_action = QAction("Add Transition", self)
        add_transition_action.triggered.connect(self.add_transition)
        toolbar.addAction(add_transition_action)

        add_final_action = QAction("Add Final Outcome", self)
        add_final_action.triggered.connect(self.add_final_outcome)
        toolbar.addAction(add_final_action)

        toolbar.addSeparator()

        delete_action = QAction("Delete Selected", self)
        delete_action.triggered.connect(self.delete_selected)
        toolbar.addAction(delete_action)

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

        root_sm_layout.addWidget(QLabel("<b>Initial State:</b>"))
        self.initial_state_combo = QComboBox()
        self.initial_state_combo.addItem("(None)")
        self.initial_state_combo.currentTextChanged.connect(self.on_initial_state_changed)
        root_sm_layout.addWidget(self.initial_state_combo)

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

    def on_initial_state_changed(self, text):
        """Handle initial state selection change."""
        if text == "(None)":
            self.initial_state = None
        else:
            self.initial_state = text

    def update_initial_state_combo(self):
        """Update the initial state combo box with available states."""
        current = self.initial_state_combo.currentText()
        self.initial_state_combo.clear()
        self.initial_state_combo.addItem("(None)")

        for state_name in self.state_nodes.keys():
            self.initial_state_combo.addItem(state_name)

        # Restore selection if still valid
        index = self.initial_state_combo.findText(current)
        if index >= 0:
            self.initial_state_combo.setCurrentIndex(index)

    def on_plugin_double_clicked(self, item: QListWidgetItem):
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok and state_name:
            self.create_state_node(state_name, plugin_info, False, False)

    def on_xml_double_clicked(self, item: QListWidgetItem):
        xml_plugin = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(
            self, "State Machine Name", "Enter state machine name:"
        )
        if ok and state_name:
            # For XML state machines, pass the plugin_info
            self.create_state_node(
                state_name, xml_plugin, True, False, xml_plugin.file_path
            )

    def create_state_node(
        self,
        name: str,
        plugin_info: PluginInfo,
        is_state_machine: bool = False,
        is_concurrence: bool = False,
        xml_file: str = None,
        outcomes: List[str] = None,
        remappings: Dict[str, str] = None,
        initial_state: str = None,
        default_outcome: str = None,
    ):
        if name in self.state_nodes:
            QMessageBox.warning(self, "Error", f"State '{name}' already exists!")
            return

        # Create node
        # XML state machines are regular StateNode (not containers)
        # Only user-created State Machines and Concurrences are containers
        if xml_file:
            # XML-based state machine - treat as regular StateNode
            node = StateNode(
                name,
                plugin_info,
                self.next_state_position.x(),
                self.next_state_position.y(),
                remappings,
            )
            node.xml_file = xml_file  # Store XML reference
        elif is_state_machine or is_concurrence:
            # User-created container (State Machine or Concurrence)
            node = ContainerStateNode(
                name,
                self.next_state_position.x(),
                self.next_state_position.y(),
                is_concurrence,
                remappings,
                outcomes,
                initial_state,
                default_outcome,
            )
        else:
            # Regular state with plugin
            node = StateNode(
                name,
                plugin_info,
                self.next_state_position.x(),
                self.next_state_position.y(),
                remappings,
            )

        self.canvas.scene.addItem(node)
        self.state_nodes[name] = node

        # Update position for next state
        self.next_state_position += QPointF(200, 150)

        # Update initial state combo
        self.update_initial_state_combo()

        # If this is the first state and no initial state is set, set it as initial
        if len(self.state_nodes) == 1 and not self.initial_state:
            self.initial_state = name
            index = self.initial_state_combo.findText(name)
            if index >= 0:
                self.initial_state_combo.setCurrentIndex(index)

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
                    is_sm,
                    is_cc,
                    xml_file,
                    outcomes,
                    remappings,
                ) = result
                self.create_state_node(
                    name,
                    plugin,
                    is_sm,
                    is_cc,
                    xml_file,
                    outcomes,
                    remappings,
                )

    def add_state_machine(self):
        """Add a new State Machine container."""
        dialog = StateMachineDialog(parent=self)

        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:  # Check if validation passed
                name, outcomes, initial_state, remappings = result
                self.create_state_node(
                    name=name,
                    plugin_info=None,
                    is_state_machine=True,
                    is_concurrence=False,
                    xml_file=None,
                    outcomes=outcomes,
                    remappings=remappings,
                    initial_state=initial_state,
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
                    xml_file=None,
                    outcomes=outcomes,
                    remappings=remappings,
                    initial_state=None,
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

        # Use appropriate dialog based on state type
        if isinstance(state_node, ContainerStateNode):
            if state_node.is_state_machine:
                # Use StateMachineDialog for State Machine containers
                dialog = StateMachineDialog(
                    name=state_node.name,
                    outcomes=(
                        list(state_node.final_outcomes.keys())
                        if state_node.final_outcomes
                        else []
                    ),
                    initial_state=state_node.initial_state,
                    remappings=state_node.remappings,
                    edit_mode=True,
                    parent=self,
                )

                if dialog.exec_():
                    result = dialog.get_state_machine_data()
                    if result:
                        name, outcomes, initial_state, remappings = result
                        state_node.remappings = remappings
                        if initial_state:
                            state_node.initial_state = initial_state
                            state_node.update_initial_state_label()
                        self.statusBar().showMessage(
                            f"Updated state machine: {name}", 2000
                        )
            elif state_node.is_concurrence:
                # Use ConcurrenceDialog for Concurrence containers
                dialog = ConcurrenceDialog(
                    name=state_node.name,
                    outcomes=(
                        list(state_node.final_outcomes.keys())
                        if state_node.final_outcomes
                        else []
                    ),
                    default_outcome=state_node.default_outcome,
                    remappings=state_node.remappings,
                    edit_mode=True,
                    parent=self,
                )

                if dialog.exec_():
                    result = dialog.get_concurrence_data()
                    if result:
                        name, outcomes, default_outcome, remappings = result
                        state_node.remappings = remappings
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
                        is_sm,
                        is_cc,
                        xml_file,
                        outcomes,
                        remappings,
                        initial_state,
                        default_outcome,
                    ) = result

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
                    is_sm,
                    is_cc,
                    xml_file,
                    outcomes,
                    remappings,
                    initial_state,
                    default_outcome,
                ) = result

                # Check if child state already exists in container
                if name in container.child_states:
                    QMessageBox.warning(
                        self, "Error", f"State '{name}' already exists in this container!"
                    )
                    return

                # Create the child state node
                if xml_file:
                    # XML-based state machine - treat as regular StateNode
                    child_node = StateNode(name, plugin, 0, 0, remappings)
                    child_node.xml_file = xml_file
                elif is_sm or is_cc:
                    # User-created container
                    child_node = ContainerStateNode(
                        name,
                        0,
                        0,
                        is_cc,
                        remappings,
                        outcomes,
                        initial_state,
                        default_outcome,
                    )
                else:
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

        if not has_outcomes or not outcomes_list:
            QMessageBox.warning(
                self,
                "Error",
                "Cannot create transitions from states without outcomes!",
            )
            return

        # Filter out already used outcomes
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
        # Check if outcome is already used
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

        # Get available targets (exclude containers as they can't be transition targets)
        available_targets = []
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
                x = 0
                y = len(selected_container.final_outcomes) * 100
                node = FinalOutcomeNode(outcome_name, x, y)
                node.parent_container = selected_container
                node.setParentItem(selected_container)
                selected_container.final_outcomes[outcome_name] = node
                selected_container.update_size()
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
                x = self.next_state_position.x() + 300
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

                # Remove node
                self.canvas.scene.removeItem(item)
                del self.state_nodes[item.name]
                self.update_initial_state_combo()  # Update combo after deletion
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
                    item.parent_container.update_size()
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
            self.next_state_position = QPointF(0, 0)
            self.root_sm_name = ""
            self.initial_state = None
            self.root_sm_name_edit.clear()
            self.update_initial_state_combo()
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

        if not self.initial_state:
            errors.append("- Initial state is not set")
        elif self.initial_state not in self.state_nodes:
            errors.append(f"- Initial state '{self.initial_state}' does not exist")

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
            try:
                self.save_to_xml(file_path)
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def save_to_xml(self, file_path: str):
        import xml.etree.ElementTree as ET
        from xml.dom import minidom

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
        if self.initial_state:
            root.set("initial_state", self.initial_state)

        # Add states
        for state_name, state_node in self.state_nodes.items():
            if state_node.is_concurrence:
                # Concurrence state
                cc_elem = ET.SubElement(root, "Concurrence")
                cc_elem.set("name", state_name)

                # Add custom outcomes if available
                if hasattr(state_node, "custom_outcomes") and state_node.custom_outcomes:
                    cc_elem.set("outcomes", " ".join(state_node.custom_outcomes))

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        remap_elem = ET.SubElement(cc_elem, "Remap")
                        remap_elem.set("from", old_key)
                        remap_elem.set("to", new_key)

            elif state_node.is_state_machine:
                # Nested state machine
                sm_elem = ET.SubElement(root, "StateMachine")
                sm_elem.set("name", state_name)

                # Set initial state if the node has one
                if hasattr(state_node, "initial_state") and state_node.initial_state:
                    sm_elem.set("initial_state", state_node.initial_state)

                # Get outcomes from XML file if available
                if hasattr(state_node, "xml_file") and state_node.xml_file:
                    # Parse XML to get outcomes
                    import xml.etree.ElementTree as ET_parse

                    tree = ET_parse.parse(state_node.xml_file)
                    xml_root = tree.getroot()
                    outcomes = xml_root.get("outcomes", "")
                    if outcomes:
                        sm_elem.set("outcomes", outcomes)
                elif (
                    hasattr(state_node, "custom_outcomes") and state_node.custom_outcomes
                ):
                    # Use custom outcomes for new state machines
                    sm_elem.set("outcomes", " ".join(state_node.custom_outcomes))

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        remap_elem = ET.SubElement(sm_elem, "Remap")
                        remap_elem.set("from", old_key)
                        remap_elem.set("to", new_key)
            else:
                state_elem = ET.SubElement(root, "State")
                state_elem.set("name", state_name)

                if state_node.plugin_info:
                    if state_node.plugin_info.plugin_type == "python":
                        state_elem.set("type", "py")
                        state_elem.set("module", state_node.plugin_info.module)
                        state_elem.set("class", state_node.plugin_info.class_name)
                    else:
                        state_elem.set("type", "cpp")
                        state_elem.set("class", state_node.plugin_info.class_name)

                # Add remappings
                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        remap_elem = ET.SubElement(state_elem, "Remap")
                        remap_elem.set("from", old_key)
                        remap_elem.set("to", new_key)

            # Add transitions
            for connection in state_node.connections:
                if connection.from_node == state_node:
                    parent_elem = (
                        state_elem
                        if not (state_node.is_state_machine or state_node.is_concurrence)
                        else (sm_elem if state_node.is_state_machine else cc_elem)
                    )
                    transition = ET.SubElement(parent_elem, "Transition")
                    transition.set("from", connection.outcome)
                    transition.set("to", connection.to_node.name)

        # Pretty print XML
        xml_str = ET.tostring(root, encoding="unicode")
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")

        # Remove extra blank lines
        pretty_xml = "\n".join([line for line in pretty_xml.split("\n") if line.strip()])

        # Write to file
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(pretty_xml)

    def load_from_xml(self, file_path: str):
        import xml.etree.ElementTree as ET

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
        initial_state = root.get("initial_state", "")

        # If no initial_state attribute, find the first State element
        if not initial_state:
            for elem in root:
                if elem.tag == "State":
                    initial_state = elem.get("name", "")
                    break

        if initial_state:
            self.initial_state = initial_state

        # Get final outcomes
        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()
            for i, outcome in enumerate(outcomes):
                x = 600
                y = i * 150
                node = FinalOutcomeNode(outcome, x, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome] = node

        # Load states
        y_offset = 0
        for elem in root:
            if elem.tag == "State":
                state_name = elem.get("name")
                state_type = elem.get("type")

                # Load remappings
                remappings = {}
                for remap in elem.findall("Remap"):
                    from_key = remap.get("from", "")
                    to_key = remap.get("to", "")
                    if from_key and to_key:
                        remappings[from_key] = to_key

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
                    node = StateNode(state_name, plugin_info, 0, y_offset, remappings)
                    self.canvas.scene.addItem(node)
                    self.state_nodes[state_name] = node
                    y_offset += 200

            elif elem.tag == "StateMachine":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")
                init_state = elem.get("initial_state", "")

                # Load remappings
                remappings = {}
                for remap in elem.findall("Remap"):
                    from_key = remap.get("from", "")
                    to_key = remap.get("to", "")
                    if from_key and to_key:
                        remappings[from_key] = to_key

                # Parse outcomes
                outcomes = outcomes_str.split() if outcomes_str else []

                # Create state machine container node
                node = ContainerStateNode(
                    state_name, 0, y_offset, False, remappings, outcomes, init_state
                )

                self.canvas.scene.addItem(node)
                self.state_nodes[state_name] = node
                y_offset += 200

            elif elem.tag == "Concurrence":
                state_name = elem.get("name")
                outcomes_str = elem.get("outcomes", "")

                # Load remappings
                remappings = {}
                for remap in elem.findall("Remap"):
                    from_key = remap.get("from", "")
                    to_key = remap.get("to", "")
                    if from_key and to_key:
                        remappings[from_key] = to_key

                # Parse outcomes
                outcomes = outcomes_str.split() if outcomes_str else []

                # Create concurrence container node
                node = ContainerStateNode(
                    state_name, 0, y_offset, True, remappings, outcomes, None
                )

                self.canvas.scene.addItem(node)
                self.state_nodes[state_name] = node
                y_offset += 200

        # Update initial state combo
        self.update_initial_state_combo()

        # Set initial state selection
        if initial_state:
            index = self.initial_state_combo.findText(initial_state)
            if index >= 0:
                self.initial_state_combo.setCurrentIndex(index)

        # Load transitions
        for elem in root:
            if elem.tag in ["State", "StateMachine", "Concurrence"]:
                state_name = elem.get("name")
                from_node = self.state_nodes.get(state_name)

                if from_node:
                    for transition in elem.findall("Transition"):
                        outcome = transition.get("from")
                        to_name = transition.get("to")

                        # Find target node
                        to_node = self.state_nodes.get(to_name)
                        if not to_node:
                            to_node = self.final_outcomes.get(to_name)

                        if to_node:
                            connection = ConnectionLine(from_node, to_node, outcome)
                            self.canvas.scene.addItem(connection)
                            self.canvas.scene.addItem(connection.arrow_head)
                            self.canvas.scene.addItem(connection.label_bg)
                            self.canvas.scene.addItem(connection.label)
                            self.connections.append(connection)
