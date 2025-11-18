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
    """Main editor window for YASMIN state machines.

    Provides a graphical interface for creating, editing, and managing
    hierarchical state machines with support for Python, C++, and XML states.

    Attributes:
        plugin_manager: Manager for loading and handling plugins.
        state_nodes: Dictionary mapping state names to StateNode objects.
        final_outcomes: Dictionary mapping outcome names to FinalOutcomeNode objects.
        connections: List of ConnectionLine objects representing transitions.
        root_sm_name: Name of the root state machine.
        start_state: Name of the initial state.
        layout_seed: Seed for deterministic layout generation.
        layout_rng: Random number generator for layout.
    """

    def __init__(self, manager: PluginManager):
        """Initialize the YASMIN Editor.

        Args:
            manager: The PluginManager instance for handling plugins.
        """
        super().__init__()
        self.setWindowTitle("YASMIN Editor")

        self.showMaximized()

        self.plugin_manager = manager
        self.state_nodes: Dict[str, StateNode] = {}
        self.final_outcomes: Dict[str, FinalOutcomeNode] = {}
        self.connections: List[ConnectionLine] = []
        self.root_sm_name = ""
        self.start_state = None

        self.layout_seed = 42
        self.layout_rng = random.Random(self.layout_seed)

        self.create_ui()

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

        left_layout.addWidget(QLabel("<b>XML State Machines:</b>"))
        self.xml_filter = QLineEdit()
        self.xml_filter.setPlaceholderText("Filter XML state machines...")
        self.xml_filter.textChanged.connect(self.filter_xml_list)
        left_layout.addWidget(self.xml_filter)
        self.xml_list = QListWidget()
        self.xml_list.itemDoubleClicked.connect(self.on_xml_double_clicked)
        left_layout.addWidget(self.xml_list)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

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

    def set_layout_seed(self):
        """Open a dialog to set the layout seed.

        Set to -1 to use non-deterministic random layout.
        """
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
        """Reset the layout RNG from the stored seed.

        If seed is None, uses module-level randomness.
        """
        if getattr(self, "layout_seed", None) is None:
            self.layout_rng = None
        else:
            self.layout_rng = random.Random(self.layout_seed)

    def reapply_layout(self):
        """Re-seed RNG and re-run the layout over the entire document."""
        self._reset_layout_rng()
        self._reorganize_all_containers()
        try:
            self._reposition_root_elements_after_resize()
        except Exception:
            pass

        for state in self.state_nodes.values():
            if hasattr(state, "connections"):
                for conn in state.connections:
                    conn.update_position()

        self.statusBar().showMessage("Reapplied layout", 2000)

    def populate_plugin_lists(self):
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

    def filter_python_list(self, text):
        """Filter Python states list based on search text.

        Args:
            text: The search text to filter by.
        """
        for i in range(self.python_list.count()):
            item = self.python_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_cpp_list(self, text):
        """Filter C++ states list based on search text.

        Args:
            text: The search text to filter by.
        """
        for i in range(self.cpp_list.count()):
            item = self.cpp_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def filter_xml_list(self, text):
        """Filter XML state machines list based on search text.

        Args:
            text: The search text to filter by.
        """
        for i in range(self.xml_list.count()):
            item = self.xml_list.item(i)
            item.setHidden(text.lower() not in item.text().lower())

    def on_root_sm_name_changed(self, text):
        """Handle root state machine name change.

        Args:
            text: The new state machine name.
        """
        self.root_sm_name = text

    def on_start_state_changed(self, text):
        """Handle initial state selection change.

        Args:
            text: The selected state name or "(None)".
        """
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
        """Handle double-click on a plugin item to add it as a state.

        Args:
            item: The list widget item that was double-clicked.
        """
        plugin_info = item.data(Qt.UserRole)
        state_name, ok = QInputDialog.getText(self, "State Name", "Enter state name:")
        if ok:
            self.create_state_node(state_name, plugin_info, False, False)

    def on_xml_double_clicked(self, item: QListWidgetItem):
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

    def get_free_position(self):
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

        root_nodes = [
            node
            for node in self.state_nodes.values()
            if not hasattr(node, "parent_container") or node.parent_container is None
        ]

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
    ):
        """Create a new state node in the canvas.

        Args:
            name: Name of the state.
            plugin_info: Plugin information for the state.
            is_state_machine: Whether this is a state machine container.
            is_concurrence: Whether this is a concurrence container.
            outcomes: List of outcome names.
            remappings: Dictionary of key remappings.
            start_state: Initial state for state machines.
            default_outcome: Default outcome for concurrences.
        """
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
        """Open dialog to add a new state to the state machine."""
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
            if result[0]:
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
            if result:
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

    def create_connection(self, from_node, to_node, outcome: str):
        """Create and add a connection to the scene.

        Args:
            from_node: The source node.
            to_node: The target node.
            outcome: The outcome name for this transition.
        """
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

    def add_final_outcome(self):
        """Add a final outcome to the state machine or selected container."""
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
        """Delete the selected items from the canvas."""
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
        """Create a new state machine, clearing the current one.

        Returns:
            bool: True if a new state machine was created, False if cancelled.
        """
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
        """Open a state machine from an XML file."""
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
            if not file_path.lower().endswith(".xml"):
                file_path += ".xml"

            try:
                self.save_to_xml(file_path)
                self.statusBar().showMessage(f"Saved: {file_path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")

    def save_to_xml(self, file_path: str):
        """Save the state machine to an XML file.

        Args:
            file_path: The path where the XML file will be saved.
        """
        sm_name = self.root_sm_name
        if not sm_name:
            sm_name, ok = QInputDialog.getText(
                self, "State Machine Name", "Enter state machine name (optional):"
            )
        else:
            ok = True

        root = ET.Element("StateMachine")
        root.set("name", sm_name)
        root.set("outcomes", " ".join(self.final_outcomes.keys()))

        if self.start_state:
            root.set("start_state", self.start_state)

        root_level_states = {
            name: node
            for name, node in self.state_nodes.items()
            if not hasattr(node, "parent_container") or node.parent_container is None
        }

        self._save_states_to_xml(root, root_level_states)

        if not file_path.lower().endswith(".xml"):
            file_path += ".xml"

        tree = ET.ElementTree(root)
        tree.write(file_path, encoding="utf-8", xml_declaration=True, pretty_print=True)

    def _save_states_to_xml(self, parent_elem, state_nodes_dict):
        """Recursively save states and their children to XML.

        Args:
            parent_elem: The parent XML element.
            state_nodes_dict: Dictionary of state names to state nodes.
        """
        for state_name, state_node in state_nodes_dict.items():
            if state_node.is_concurrence:
                # Concurrence state
                cc_elem = ET.SubElement(parent_elem, "Concurrence")
                cc_elem.set("name", state_name)

                if hasattr(state_node, "default_outcome") and state_node.default_outcome:
                    cc_elem.set("default_outcome", state_node.default_outcome)

                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    cc_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:
                            remap_elem = ET.SubElement(cc_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                if hasattr(state_node, "child_states") and state_node.child_states:
                    self._save_states_to_xml(cc_elem, state_node.child_states)

                self._save_transitions(cc_elem, state_node)

                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    for outcome_node in state_node.final_outcomes.values():
                        self._save_transitions(cc_elem, outcome_node)

            elif state_node.is_state_machine:
                sm_elem = ET.SubElement(parent_elem, "StateMachine")
                sm_elem.set("name", state_name)

                if hasattr(state_node, "start_state") and state_node.start_state:
                    sm_elem.set("start_state", state_node.start_state)

                if hasattr(state_node, "final_outcomes") and state_node.final_outcomes:
                    sm_elem.set("outcomes", " ".join(state_node.final_outcomes.keys()))

                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:
                            remap_elem = ET.SubElement(sm_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                if hasattr(state_node, "child_states") and state_node.child_states:
                    self._save_states_to_xml(sm_elem, state_node.child_states)

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

                if state_node.remappings:
                    for old_key, new_key in state_node.remappings.items():
                        if old_key and new_key:
                            remap_elem = ET.SubElement(state_elem, "Remap")
                            remap_elem.set("old", old_key)
                            remap_elem.set("new", new_key)

                self._save_transitions(state_elem, state_node)

    def _save_transitions(self, parent_elem, state_node):
        """Save transitions for a state node.

        Args:
            parent_elem: The parent XML element.
            state_node: The state node whose transitions to save.
        """
        for connection in state_node.connections:
            if connection.from_node == state_node:
                transition = ET.SubElement(parent_elem, "Transition")
                if connection.outcome:
                    transition.set("from", connection.outcome)
                if connection.to_node and connection.to_node.name:
                    transition.set("to", connection.to_node.name)

    def load_from_xml(self, file_path: str):
        """Load a state machine from an XML file.

        Args:
            file_path: The path to the XML file to load.
        """
        if not self.new_state_machine():
            return

        tree = ET.parse(file_path)
        root = tree.getroot()

        sm_name = root.get("name", "")
        if sm_name:
            self.root_sm_name = sm_name
            self.root_sm_name_edit.setText(sm_name)

        start_state = root.get("start_state", "")

        if not start_state:
            for elem in root:
                if elem.tag == "State":
                    start_state = elem.get("name", "")
                    break

        if start_state:
            self.start_state = start_state

        self._load_states_from_xml(root, None)

        outcomes_str = root.get("outcomes", "")
        if outcomes_str:
            outcomes = outcomes_str.split()
            for i, outcome in enumerate(outcomes):
                y = 200 + i * 120
                node = FinalOutcomeNode(outcome, 800, y)
                self.canvas.scene.addItem(node)
                self.final_outcomes[outcome] = node

        self.update_start_state_combo()

        if start_state:
            index = self.start_state_combo.findText(start_state)
            if index >= 0:
                self.start_state_combo.setCurrentIndex(index)

        self._load_transitions_from_xml(root, None)

        self._reorganize_all_containers()

        QApplication.processEvents()

        self._reposition_root_elements_after_resize()

        for state in self.state_nodes.values():
            if hasattr(state, "connections"):
                for conn in state.connections:
                    conn.update_position()

        for outcome in self.final_outcomes.values():
            if hasattr(outcome, "connections"):
                for conn in outcome.connections:
                    conn.update_position()

        for _ in range(3):
            self.canvas.scene.update()
            QApplication.processEvents()

    def _reorganize_all_containers(self):
        """Reorganize all containers and their children after loading.

        Uses force-directed layout for graph positioning.
        """
        self._reset_layout_rng()
        all_containers = []
        for node in self.state_nodes.values():
            if isinstance(node, ContainerStateNode):
                all_containers.append(node)

        def get_nesting_depth(container):
            depth = 0
            current = container
            while hasattr(current, "parent_container") and current.parent_container:
                depth += 1
                current = current.parent_container
            return depth

        all_containers.sort(key=get_nesting_depth, reverse=True)

        for container in all_containers:
            self._layout_container_force_directed(container)

        for container in all_containers:
            container.prepareGeometryChange()
            container.auto_resize_for_children()

        for container in all_containers:
            container.prepareGeometryChange()
            container.update()

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

        for _ in range(4):
            self.canvas.scene.update()
            QApplication.processEvents()

    def _layout_container_force_directed(self, container: ContainerStateNode):
        """Layout children within a container using Fruchterman–Reingold algorithm.

        Uses physical simulation with Fruchterman-Reingold algorithm:
        - Repulsive forces between all nodes (prevent overlap)
        - Attractive forces along edges (keep connected nodes close)
        - Boundary forces (keep nodes within container)

        Args:
            container: The container state node to layout.
        """
        if not container.child_states and not container.final_outcomes:
            return

        nodes = list(container.child_states.values())
        final_outcomes = list(container.final_outcomes.values())

        if not nodes:
            rect = container.rect()
            y_start = rect.top() + 140
            for i, outcome in enumerate(final_outcomes):
                outcome.setPos(rect.left() + 120, y_start + i * 120)
            container.auto_resize_for_children()
            return

        all_nodes = nodes + final_outcomes
        graph = {node: [] for node in all_nodes}

        for node in nodes:
            if hasattr(node, "connections"):
                for conn in node.connections:
                    if conn.from_node == node:
                        if conn.to_node in all_nodes:
                            graph[node].append(conn.to_node)

        self._force_directed_layout(container, all_nodes, graph)

    def _force_directed_layout(self, container, nodes, graph):
        """Fruchterman-Reingold force-directed layout algorithm.

        Simulates physical forces:
        - Repulsion: All nodes repel each other (prevents overlap)
        - Attraction: Connected nodes attract each other (keeps graph connected)
        - Boundary: Nodes are pushed back if they leave the container

        Args:
            container: The container node.
            nodes: List of all nodes to position.
            graph: Adjacency dictionary mapping nodes to their neighbors.
        """
        import math

        rng = self.layout_rng if getattr(self, "layout_rng", None) is not None else random

        rect = container.rect()

        MARGIN_X = rect.width() * 0.05
        MARGIN_Y_TOP = rect.height() * 0.12
        MARGIN_Y_BOTTOM = rect.height() * 0.05

        area_width = rect.width() - (2 * MARGIN_X)
        area_height = rect.height() - MARGIN_Y_TOP - MARGIN_Y_BOTTOM

        node_dimensions = {}
        for node in nodes:
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                w, h = node.rect().width(), node.rect().height()
            else:
                w, h = node.boundingRect().width(), node.boundingRect().height()
            node_dimensions[node] = (w, h)

        positions = {}
        for node in nodes:
            current_pos = node.pos()
            if (
                rect.left() < current_pos.x() < rect.right()
                and rect.top() < current_pos.y() < rect.bottom()
            ):
                positions[node] = [current_pos.x(), current_pos.y()]
            else:
                x = rect.left() + MARGIN_X + rng.random() * area_width
                y = rect.top() + MARGIN_Y_TOP + rng.random() * area_height
                positions[node] = [x, y]

        area = area_width * area_height
        k = math.sqrt(area / len(nodes))

        ITERATIONS = 100
        initial_temp = area_width / 10
        temp = initial_temp

        for iteration in range(ITERATIONS):
            displacement = {node: [0.0, 0.0] for node in nodes}

            for i, node1 in enumerate(nodes):
                for node2 in nodes[i + 1 :]:
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

            for node in nodes:
                disp_x = displacement[node][0]
                disp_y = displacement[node][1]

                disp_length = math.sqrt(disp_x**2 + disp_y**2)
                if disp_length < 0.01:
                    disp_length = 0.01

                limited_disp = min(disp_length, temp) / disp_length

                positions[node][0] += disp_x * limited_disp
                positions[node][1] += disp_y * limited_disp

                node_w, node_h = node_dimensions[node]
                positions[node][0] = max(
                    rect.left() + MARGIN_X,
                    min(positions[node][0], rect.right() - MARGIN_X - node_w),
                )
                positions[node][1] = max(
                    rect.top() + MARGIN_Y_TOP,
                    min(positions[node][1], rect.bottom() - MARGIN_Y_BOTTOM - node_h),
                )

            temp = initial_temp * (1 - iteration / ITERATIONS)

        for node, (x, y) in positions.items():
            node.setPos(x, y)

        container.auto_resize_for_children()

    def _count_crossings(self, layers, graph):
        """Count the total number of edge crossings between adjacent layers.

        Args:
            layers: List of layers with nodes.
            graph: Forward adjacency graph.

        Returns:
            int: Total number of crossings.
        """
        total_crossings = 0

        for i in range(len(layers) - 1):
            layer1 = layers[i]
            layer2 = layers[i + 1]

            pos1 = {node: idx for idx, node in enumerate(layer1)}
            pos2 = {node: idx for idx, node in enumerate(layer2)}

            edges = []
            for node in layer1:
                for neighbor in graph.get(node, []):
                    if neighbor in pos2:
                        edges.append((pos1[node], pos2[neighbor]))

            for i in range(len(edges)):
                for j in range(i + 1, len(edges)):
                    a1, a2 = edges[i]
                    b1, b2 = edges[j]
                    if (a1 < b1 and a2 > b2) or (a1 > b1 and a2 < b2):
                        total_crossings += 1

        return total_crossings

    def _reposition_root_elements_after_resize(self):
        """Reposition root-level elements using force-directed layout.

        Ensures optimal spacing and prevents overlapping.
        Includes final outcomes in the graph layout.
        """
        root_nodes = []
        for node in self.state_nodes.values():
            if not hasattr(node, "parent_container") or node.parent_container is None:
                root_nodes.append(node)

        root_final_outcomes = list(self.final_outcomes.values())

        if not root_nodes and not root_final_outcomes:
            return

        all_nodes = root_nodes + root_final_outcomes
        graph = {node: [] for node in all_nodes}
        reverse_graph = {node: [] for node in all_nodes}

        for node in root_nodes:
            if hasattr(node, "connections"):
                for conn in node.connections:
                    if conn.from_node == node:
                        if conn.to_node in all_nodes:
                            graph[node].append(conn.to_node)
                            reverse_graph[conn.to_node].append(node)

        total_edges = sum(len(neighbors) for neighbors in graph.values())
        if total_edges == 0 and len(all_nodes) > 1:
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

        self._force_directed_layout_root(all_nodes, graph)

    def _force_directed_layout_root(self, nodes, graph):
        """Force-directed layout for root-level nodes.

        Uses Fruchterman-Reingold algorithm adapted for the root canvas.

        Args:
            nodes: List of all root-level nodes.
            graph: Adjacency dictionary.
        """
        import math

        rng = self.layout_rng if getattr(self, "layout_rng", None) is not None else random

        viewport_rect = self.canvas.viewport().rect()
        viewport_width = viewport_rect.width()
        viewport_height = viewport_rect.height()

        MARGIN_X = viewport_width * 0.1
        MARGIN_Y = viewport_height * 0.1

        area_width = viewport_width - (2 * MARGIN_X)
        area_height = viewport_height - (2 * MARGIN_Y)

        node_dimensions = {}
        for node in nodes:
            if isinstance(node, ContainerStateNode):
                node.prepareGeometryChange()
                w, h = node.rect().width(), node.rect().height()
            else:
                w, h = node.boundingRect().width(), node.boundingRect().height()
            node_dimensions[node] = (w, h)

        positions = {}
        for node in nodes:
            current_pos = node.scenePos()
            if (
                MARGIN_X < current_pos.x() < viewport_width - MARGIN_X
                and MARGIN_Y < current_pos.y() < viewport_height - MARGIN_Y
            ):
                positions[node] = [current_pos.x(), current_pos.y()]
            else:
                x = MARGIN_X + rng.random() * area_width
                y = MARGIN_Y + rng.random() * area_height
                positions[node] = [x, y]

        area = area_width * area_height
        k = math.sqrt(area / len(nodes)) * 1.5

        ITERATIONS = 150
        initial_temp = area_width / 8
        temp = initial_temp

        for iteration in range(ITERATIONS):
            displacement = {node: [0.0, 0.0] for node in nodes}

            for i, node1 in enumerate(nodes):
                for node2 in nodes[i + 1 :]:
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

            for node in nodes:
                disp_x = displacement[node][0]
                disp_y = displacement[node][1]

                disp_length = math.sqrt(disp_x**2 + disp_y**2)
                if disp_length < 0.01:
                    disp_length = 0.01

                limited_disp = min(disp_length, temp) / disp_length

                positions[node][0] += disp_x * limited_disp
                positions[node][1] += disp_y * limited_disp

                node_w, node_h = node_dimensions[node]
                positions[node][0] = max(
                    MARGIN_X, min(positions[node][0], viewport_width - MARGIN_X - node_w)
                )
                positions[node][1] = max(
                    MARGIN_Y, min(positions[node][1], viewport_height - MARGIN_Y - node_h)
                )

            temp = initial_temp * (1 - iteration / ITERATIONS)

        for node, (x, y) in positions.items():
            node.setPos(x, y)

        for node in nodes:
            if hasattr(node, "connections"):
                for conn in node.connections:
                    conn.update_position()

    def _load_states_from_xml(self, parent_elem, parent_container):
        """Recursively load states from XML, handling nested containers.

        Args:
            parent_elem: The parent XML element.
            parent_container: The parent container node (None for root).
        """
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
        """Load remappings from an XML element.

        Args:
            elem: The XML element to load remappings from.

        Returns:
            dict: Dictionary of remappings (old -> new).
        """
        remappings = {}
        for remap in elem.findall("Remap"):
            from_key = remap.get("old", "")
            to_key = remap.get("new", "")
            if from_key and to_key:
                remappings[from_key] = to_key
        return remappings

    def _load_transitions_from_xml(self, parent_elem, parent_container):
        """Recursively load transitions from XML.

        Args:
            parent_elem: The parent XML element.
            parent_container: The parent container node (None for root).
        """
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
                    final_outcome_names = set()
                    if elem.tag in ["StateMachine", "Concurrence"] and hasattr(
                        from_node, "final_outcomes"
                    ):
                        final_outcome_names = set(from_node.final_outcomes.keys())

                    for transition in elem.findall("Transition"):
                        outcome = transition.get("from")
                        to_name = transition.get("to")

                        is_from_final_outcome = outcome in final_outcome_names

                        if is_from_final_outcome:
                            from_outcome = from_node.final_outcomes[outcome]

                            to_node = None

                            if parent_container is None:
                                to_node = self.state_nodes.get(to_name)
                                if not to_node:
                                    to_node = self.final_outcomes.get(to_name)
                            elif to_name in parent_container.final_outcomes:
                                to_node = parent_container.final_outcomes[to_name]
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
                            to_node = None

                            if parent_container is None:
                                to_node = self.state_nodes.get(to_name)
                                if not to_node:
                                    to_node = self.final_outcomes.get(to_name)
                            else:
                                if to_name in parent_container.final_outcomes:
                                    to_node = parent_container.final_outcomes[to_name]
                                else:
                                    full_to_name = f"{parent_container.name}.{to_name}"
                                    to_node = self.state_nodes.get(full_to_name)

                            if to_node:
                                connection = ConnectionLine(from_node, to_node, outcome)
                                self.canvas.scene.addItem(connection)
                                self.canvas.scene.addItem(connection.arrow_head)
                                self.canvas.scene.addItem(connection.label_bg)
                                self.canvas.scene.addItem(connection.label)
                                self.connections.append(connection)

                if elem.tag in ["StateMachine", "Concurrence"]:
                    if parent_container is None:
                        container = self.state_nodes.get(state_name)
                    else:
                        full_name = f"{parent_container.name}.{state_name}"
                        container = self.state_nodes.get(full_name)

                    if container:
                        self._load_transitions_from_xml(elem, container)
