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

import os
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QInputDialog,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QSplitter,
    QWidget,
)
from yasmin_plugins_manager.plugin_manager import PluginInfo

from yasmin_editor.editor_gui.clipboard_model import is_container_empty
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.editor_action_state import (
    build_editor_action_enabled_map,
    toolbar_menu_enabled,
)
from yasmin_editor.editor_gui.dialog_result_adapters import (
    build_concurrence_kwargs,
    build_plugin_state_kwargs,
    build_state_machine_kwargs,
)
from yasmin_editor.editor_gui.final_outcome_ops import ensure_final_outcome_alias
from yasmin_editor.editor_gui.dialogs.concurrence_dialog import ConcurrenceDialog
from yasmin_editor.editor_gui.dialogs.outcome_description_dialog import (
    OutcomeDescriptionDialog,
)
from yasmin_editor.editor_gui.dialogs.state_machine_dialog import StateMachineDialog
from yasmin_editor.editor_gui.dialogs.state_properties_dialog import StatePropertiesDialog
from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.nodes.state_node import StateNode
from yasmin_editor.editor_gui.nodes.text_block_node import TextBlockNode
from yasmin_editor.editor_gui.scene_selection import collect_scene_selection
from yasmin_editor.editor_gui.transition_rules import (
    TransitionRuleError,
    get_available_transition_outcomes,
    validate_drag_target,
)
from yasmin_editor.editor_gui.ui.docks import build_clipboard_dock
from yasmin_editor.editor_gui.ui.help_dialog import show_help_dialog
from yasmin_editor.editor_gui.ui.layout_contract import apply_splitter_layout
from yasmin_editor.editor_gui.ui.menus import build_menu_bar
from yasmin_editor.editor_gui.ui.panels import build_right_panel
from yasmin_editor.editor_gui.ui.plugin_lists import (
    filter_list_widget,
    populate_plugin_lists as populate_plugin_list_widgets,
)
from yasmin_editor.editor_gui.ui.sidebars import build_left_panel
from yasmin_editor.editor_gui.ui.toolbar import build_toolbar
from yasmin_editor.editor_gui.ui.toolbar_config import (
    ADD_TOOLBAR_MENU,
    SELECTION_TOOLBAR_MENU,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state_machine import StateMachine


class EditorUiMixin:
    """Mixin for editor functionality split from the main window."""

    def on_canvas_selection_changed(self) -> None:
        """Refresh scene highlighting and selection-dependent action states."""

        self.refresh_visual_highlighting()
        self.update_editor_action_states()

    def update_editor_action_states(self) -> None:
        """Update actions and toolbar buttons that depend on editor context."""

        selected_items = []
        if hasattr(self, "canvas") and hasattr(self.canvas, "scene"):
            selected_items = self.canvas.scene.selectedItems()
        selection = collect_scene_selection(selected_items)
        enabled_map = build_editor_action_enabled_map(
            read_only_mode=self.is_read_only_mode(),
            has_selection=not selection.is_empty,
            has_state_selection=bool(selection.states),
            clipboard_has_content=not is_container_empty(self.clipboard_model),
        )
        for action_name, enabled in enabled_map.items():
            action = getattr(self, action_name, None)
            if action is not None:
                action.setEnabled(enabled)

        button_specs = (
            ("toolbar_add_menu_button", ADD_TOOLBAR_MENU),
            ("toolbar_selection_menu_button", SELECTION_TOOLBAR_MENU),
        )
        for button_name, spec in button_specs:
            button = getattr(self, button_name, None)
            if button is not None:
                button.setEnabled(
                    toolbar_menu_enabled(spec.action_attributes, enabled_map)
                )

        self.update_history_actions()

    def populate_plugin_lists(self) -> None:
        """Populate the plugin lists with available Python, C++, and XML states."""
        populate_plugin_list_widgets(self)

    def filter_list(self, list_widget: QListWidget, text: str) -> None:
        """Filter a list widget based on search text."""
        filter_list_widget(list_widget, text)

    def show_help(self) -> None:
        """Display help dialog with usage instructions."""
        show_help_dialog(self)

    def new_state_machine(self) -> bool:
        """Create a new state machine, clearing the current one."""
        if self.runtime_mode_enabled:
            self._show_read_only_message()
            return False

        if not self.maybe_save_document_changes("creating a new state machine"):
            return False

        self.reset_editor_state()
        self.reset_history()
        self.statusBar().showMessage("New state machine created", 2000)
        return True

    def _open_state_machine_file(self, file_path: str) -> bool:
        """Open one XML file into the editor after loading and validation."""

        if self.runtime_mode_enabled:
            self._show_read_only_message()
            return False

        if not file_path:
            return False

        if not os.path.exists(file_path):
            self.remove_recent_file(file_path)
            QMessageBox.warning(
                self,
                "File Not Found",
                f"The file does not exist anymore:\n{file_path}",
            )
            return False

        try:
            model = self.model_adapter.load_external_xml_model(file_path)
        except Exception as error:
            QMessageBox.critical(self, "Error", f"Failed to open file: {str(error)}")
            return False

        if not self.maybe_save_document_changes("opening another state machine"):
            return False

        self.model_adapter.apply_root_model(
            model,
            current_file_path=file_path,
            fit_view=True,
        )
        self.register_recent_file(file_path)
        self.statusBar().showMessage(f"Opened: {file_path}", 3000)
        return True

    def open_state_machine(self) -> None:
        """Prompt for and open a state machine XML file."""

        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open State Machine", "", "XML Files (*.xml)"
        )
        if not file_path:
            return
        self._open_state_machine_file(file_path)

    def open_recent_state_machine(self, file_path: str) -> None:
        """Open one file selected from the recent-documents submenu."""

        self._open_state_machine_file(file_path)

    def create_ui(self) -> None:
        """Create and setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        build_menu_bar(self)
        build_toolbar(self)

        left_panel = build_left_panel(self)
        right_panel = build_right_panel(self)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        apply_splitter_layout(splitter)

        build_clipboard_dock(self)
        self.toggle_clipboard_panel(visible=False)
        self.update_editor_action_states()

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
            declared_parent_parameters=self.parameters_to_dicts(
                self.current_container_model.parameters
            ),
        )

        if dialog.exec_():
            result = dialog.get_state_data()
            if result[0]:
                self.create_state_node(**build_plugin_state_kwargs(result))

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
        dialog = StatePropertiesDialog(
            available_plugins=all_plugins,
            parent=self,
            declared_parent_parameters=self.parameters_to_dicts(
                self.current_container_model.parameters
            ),
        )
        if dialog.exec_():
            result = dialog.get_state_data()
            if result[0]:
                self.create_state_node(**build_plugin_state_kwargs(result))

    def add_state_machine(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        dialog = StateMachineDialog(parent=self)
        if dialog.exec_():
            result = dialog.get_state_machine_data()
            if result:
                self.create_state_node(**build_state_machine_kwargs(result))

    def add_concurrence(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        dialog = ConcurrenceDialog(parent=self)
        if dialog.exec_():
            result = dialog.get_concurrence_data()
            if result:
                self.create_state_node(**build_concurrence_kwargs(result))

    def add_text_block(self) -> None:
        """Add a free-form text block to the current container."""
        if self.is_read_only_mode():
            self._show_read_only_message()
            return

        node = self.add_text_block_model()
        self.start_pending_node_placement(node)

    def add_final_outcome(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return

        current_model = self.current_container_model
        outcome_name, ok = QInputDialog.getText(
            self, "Final Outcome", "Enter final outcome name:"
        )
        outcome_name = outcome_name.strip()
        if not (ok and outcome_name):
            return

        if outcome_name in current_model.states:
            QMessageBox.warning(
                self,
                "Error",
                f"Final outcome '{outcome_name}' conflicts with an existing state in this container!",
            )
            return

        scene_pos = self.canvas.get_preferred_placement_scene_pos()
        result = ensure_final_outcome_alias(
            current_model,
            outcome_name,
            float(scene_pos.x()),
            float(scene_pos.y()),
        )
        nodes = self.model_adapter.create_final_outcome_views(result.outcome)
        node = next(
            (item for item in nodes if item.instance_id == result.instance_id), None
        )
        if node is None:
            node = self.get_primary_final_outcome_view(outcome_name)

        self.update_start_state_combo()
        self.refresh_connection_port_visibility()
        if node is not None:
            self.start_pending_node_placement(node)

    def create_connection_from_drag(
        self, from_node: StateNode, to_node: StateNode
    ) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return
        current_model = self.current_container_model

        try:
            validate_drag_target(
                current_model,
                from_is_final_outcome=isinstance(from_node, FinalOutcomeNode),
                to_is_final_outcome=isinstance(to_node, FinalOutcomeNode),
            )
            used_outcomes = {
                transition.source_outcome
                for transition in current_model.transitions.get(from_node.name, [])
            }
            available_outcomes = get_available_transition_outcomes(
                current_model,
                [outcome.name for outcome in from_node.model.outcomes],
                used_outcomes,
            )
        except TransitionRuleError as exc:
            QMessageBox.warning(self, getattr(exc, "title", "Error"), str(exc))
            return

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
        self.record_history_checkpoint()

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
        self.record_history_checkpoint()

    def delete_selected(self) -> None:
        if self.is_read_only_mode():
            self._show_read_only_message()
            return

        selection = collect_scene_selection(self.canvas.scene.selectedItems())
        if selection.is_empty:
            return

        for connection in selection.connections:
            if connection in self.connections:
                self.delete_connection_item(connection)

        for text_block in selection.text_blocks:
            if text_block in self.text_blocks:
                self.delete_text_block_item(text_block)

        for outcome in selection.final_outcomes:
            if getattr(outcome, "instance_id", "") in self.final_outcomes:
                self.delete_final_outcome_item(outcome)

        for state in selection.states:
            if state.name in self.state_nodes:
                self.delete_state_item(state)

        self.record_history_checkpoint()

    def edit_final_outcome(self, outcome_node: Optional[FinalOutcomeNode] = None) -> None:
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

            if new_name != outcome_node.name and self.has_final_outcome_name_conflict(
                new_name,
                current_name=outcome_node.name,
            ):
                QMessageBox.warning(
                    self,
                    "Error",
                    f"Final outcome '{new_name}' conflicts with an existing state or final outcome in this container!",
                )
                return

            if new_name != outcome_node.name:
                try:
                    self.rename_final_outcome(outcome_node, new_name)
                except ValueError as exc:
                    QMessageBox.warning(self, "Error", str(exc))
                    return

            outcome_node.description = dialog.get_description()
            outcome_node.update_tooltip()
            self.statusBar().showMessage(
                f"Updated final outcome: {outcome_node.name}",
                2000,
            )
            self.record_history_checkpoint()

    def edit_current_container(self) -> None:
        model = self.current_container_model
        input_keys, output_keys = self._collect_container_key_lists(model)
        parent_model = self.current_parent_model
        dialog = StatePropertiesDialog(
            state_name=model.name,
            plugin_info=None,
            available_plugins=[],
            remappings=dict(model.remappings),
            parameter_overwrites=(
                self.get_parameter_overwrites_for_child(parent_model, model)
                if parent_model is not None
                else []
            ),
            declared_parent_parameters=(
                self.parameters_to_dicts(parent_model.parameters)
                if parent_model is not None
                else []
            ),
            outcomes=[outcome.name for outcome in model.outcomes],
            edit_mode=True,
            parent=self,
            description=getattr(model, "description", ""),
            defaults=[],
            fallback_input_keys=input_keys,
            fallback_output_keys=output_keys,
            fallback_parameters=self.parameters_to_dicts(model.parameters),
            container_kind=(
                "Concurrence" if isinstance(model, Concurrence) else "State Machine"
            ),
            readonly=self.is_read_only_mode(),
            enable_parameter_overwrites=parent_model is not None,
        )
        if self.is_read_only_mode():
            dialog.exec_()
            return

        if dialog.exec_():
            result = dialog.get_state_data()
            if result and result[0]:
                (
                    name,
                    plugin,
                    outcomes,
                    remappings,
                    description,
                    defaults,
                    parameter_overwrites,
                ) = result
                old_name = model.name
                name = normalize_container_name(name)
                if parent_model is None:
                    model.name = name
                    if isinstance(model, StateMachine):
                        model.rename_transition_owner(old_name, name)
                else:
                    if has_container_name_conflict(
                        name,
                        current_name=old_name,
                        sibling_state_names=parent_model.states.keys(),
                        sibling_outcome_names=[
                            outcome.name for outcome in parent_model.outcomes
                        ],
                    ):
                        QMessageBox.warning(
                            self,
                            "Error",
                            f"State '{name}' conflicts with an existing state or final outcome in this container!",
                        )
                        return
                    if name != old_name:
                        parent_model.rename_state(old_name, name)
                model.description = description
                model.remappings.clear()
                model.remappings.update(remappings)
                if parent_model is not None:
                    self.apply_parameter_overwrites(
                        parent_model,
                        model,
                        parameter_overwrites,
                    )
                self.update_container_controls()
                self.refresh_breadcrumbs()
                self.refresh_blackboard_keys_list()
                self.statusBar().showMessage(
                    f"Updated {'concurrence' if isinstance(model, Concurrence) else 'state machine'}: {name}",
                    2000,
                )
                self.record_history_checkpoint()

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
            input_keys, output_keys = self._collect_container_key_lists(state_node.model)
            dialog = StatePropertiesDialog(
                state_name=state_node.name,
                plugin_info=None,
                available_plugins=[],
                remappings=state_node.remappings,
                parameter_overwrites=self.get_parameter_overwrites_for_child(
                    self.current_container_model,
                    state_node.model,
                ),
                declared_parent_parameters=self.parameters_to_dicts(
                    self.current_container_model.parameters
                ),
                outcomes=[outcome.name for outcome in state_node.model.outcomes],
                edit_mode=True,
                parent=self,
                description=getattr(state_node, "description", ""),
                defaults=getattr(state_node, "defaults", []),
                fallback_input_keys=input_keys,
                fallback_output_keys=output_keys,
                fallback_parameters=self.parameters_to_dicts(state_node.model.parameters),
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
                    (
                        name,
                        plugin,
                        outcomes,
                        remappings,
                        description,
                        defaults,
                        parameter_overwrites,
                    ) = result
                    if not self.apply_common_state_updates(
                        state_node,
                        name,
                        remappings,
                        description,
                        defaults,
                        parameter_overwrites,
                    ):
                        return
                    self.sync_blackboard_keys()
                    self.statusBar().showMessage(
                        f"Updated {'concurrence' if state_node.is_concurrence else 'state machine'}: {name}",
                        2000,
                    )
                    self.record_history_checkpoint()
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
            parameter_overwrites=self.get_parameter_overwrites_for_child(
                self.current_container_model,
                state_node.model,
            ),
            declared_parent_parameters=self.parameters_to_dicts(
                self.current_container_model.parameters
            ),
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
                (
                    name,
                    plugin,
                    outcomes,
                    remappings,
                    description,
                    defaults,
                    parameter_overwrites,
                ) = result
                if not self.apply_common_state_updates(
                    state_node,
                    name,
                    remappings,
                    description,
                    defaults,
                    parameter_overwrites,
                ):
                    return
                state_node.model.outcomes = [Outcome(name=item) for item in outcomes]
                self.sync_blackboard_keys()
                self.refresh_connection_port_visibility()
                self.statusBar().showMessage(f"Updated state: {name}", 2000)
                self.record_history_checkpoint()
