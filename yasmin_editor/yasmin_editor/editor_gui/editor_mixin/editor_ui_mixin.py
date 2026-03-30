# Copyright (C) 2026 Maik Knof
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

from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QDialog, QFileDialog, QHBoxLayout,
                             QInputDialog, QListWidget, QListWidgetItem,
                             QMessageBox, QPushButton, QSplitter, QTextBrowser,
                             QVBoxLayout, QWidget)
from yasmin_plugins_manager.plugin_manager import PluginInfo

from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.dialogs.concurrence_dialog import \
    ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.outcome_description_dialog import \
    OutcomeDescriptionDialog
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import \
    StateMachineDialog
from yasmin_editor.editor_gui.dialogs.state_properties_dialog import \
    StatePropertiesDialog
from yasmin_editor.editor_gui.nodes.container_state_node import \
    ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.ui.panels import build_right_panel
from yasmin_editor.editor_gui.ui.sidebars import build_left_panel
from yasmin_editor.editor_gui.ui.toolbar import build_toolbar
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state_machine import StateMachine

class EditorUiMixin:
    """Mixin for editor functionality split from the main window."""

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

    def create_ui(self) -> None:
        """Create and setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        build_toolbar(self)

        left_panel = build_left_panel(self)
        right_panel = build_right_panel(self)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([300, 1000])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.statusBar()
        self.refresh_breadcrumbs()
        self.update_container_controls()
        self.update_runtime_actions()

    def _open_add_state_dialog_for_plugin(self, plugin_info: PluginInfo) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return

        all_plugins = (
            self.plugin_manager.python_plugins
            + self.plugin_manager.cpp_plugins
            + self.plugin_manager.xml_files
        )

        dialog = StatePropertiesDialog(
            plugin_info=plugin_info,
            available_plugins=all_plugins,
            parent=self,
        )

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

    def on_plugin_double_clicked(self, item: QListWidgetItem) -> None:
        plugin_info = item.data(Qt.UserRole)
        if plugin_info is None:
            return
        self._open_add_state_dialog_for_plugin(plugin_info)

    def on_xml_double_clicked(self, item: QListWidgetItem) -> None:
        xml_plugin = item.data(Qt.UserRole)
        if xml_plugin is None:
            return
        self._open_add_state_dialog_for_plugin(xml_plugin)

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

            scene_pos = self.canvas.get_preferred_placement_scene_pos()
            model = Outcome(name=outcome_name)
            current_model.add_outcome(model)
            current_model.layout.set_outcome_position(
                outcome_name,
                float(scene_pos.x()),
                float(scene_pos.y()),
            )
            node = self.model_adapter.create_final_outcome_view(
                model,
                x=float(scene_pos.x()),
                y=float(scene_pos.y()),
            )

            if isinstance(current_model, Concurrence):
                if (
                    len(current_model.outcomes) == 1
                    and not current_model.default_outcome
                ):
                    current_model.default_outcome = outcome_name

            self.update_start_state_combo()
            self.refresh_connection_port_visibility()
            self.start_pending_node_placement(node)

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
            new_name = dialog.get_outcome_name()
            if not new_name:
                QMessageBox.warning(
                    self,
                    "Error",
                    "Final outcome name must not be empty!",
                )
                return

            if new_name != outcome_node.name and new_name in self.final_outcomes:
                QMessageBox.warning(
                    self,
                    "Error",
                    f"Final outcome '{new_name}' already exists in this container!",
                )
                return

            if new_name != outcome_node.name:
                self.rename_final_outcome(outcome_node, new_name)

            outcome_node.description = dialog.get_description()
            outcome_node.update_tooltip()
            self.statusBar().showMessage(
                f"Updated final outcome: {outcome_node.name}",
                2000,
            )

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
