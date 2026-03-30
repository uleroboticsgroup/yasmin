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
from typing import List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtWidgets import QLabel, QMessageBox, QPushButton
from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_line import ConnectionLine
from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
from yasmin_editor.editor_gui.state_node import StateNode
from yasmin_editor.model.concurrence import Concurrence


class EditorCanvasMixin:
    """Mixin for editor functionality split from the main window."""

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
