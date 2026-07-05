# Copyright (C) 2025 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Union

from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets, exec_menu
from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.orthogonal_state import OrthogonalState
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

if TYPE_CHECKING:
    from yasmin_plugins_manager.plugin_info import PluginInfo


class ContainerStateNode(BaseNodeMixin, QtWidgets.QGraphicsRectItem):
    """Graphical representation of a nested State Machine or Concurrence."""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        is_concurrence: bool = False,
        is_orthogonal: bool = False,
        remappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
        start_state: Optional[str] = None,
        default_outcome: Optional[str] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
        model: Optional[Union[StateMachine, Concurrence, OrthogonalState, State]] = None,
        state_kind_label: Optional[str] = None,
        is_xml_reference: bool = False,
    ) -> None:
        super().__init__(-65, -40, 130, 80)
        if model is not None:
            pass
        elif is_orthogonal:
            model = OrthogonalState(
                name=name,
                description=description,
                default_outcome=default_outcome,
                remappings=dict(remappings or {}),
                outcomes=[Outcome(name=item) for item in outcomes or []],
            )
        elif is_concurrence:
            model = Concurrence(
                name=name,
                description=description,
                default_outcome=default_outcome,
                remappings=dict(remappings or {}),
                outcomes=[Outcome(name=item) for item in outcomes or []],
            )
        else:
            model = StateMachine(
                name=name,
                description=description,
                start_state=start_state,
                remappings=dict(remappings or {}),
                outcomes=[Outcome(name=item) for item in outcomes or []],
            )
        self.model: Union[StateMachine, Concurrence, OrthogonalState, State] = model
        self.plugin_info: Optional["PluginInfo"] = None
        self.is_state_machine = isinstance(self.model, StateMachine)
        self.is_concurrence = isinstance(self.model, Concurrence)
        self.is_orthogonal = isinstance(self.model, OrthogonalState)
        self.is_xml_reference = bool(is_xml_reference)
        type_label = None
        if self.is_orthogonal:
            type_label = "ORTHOGONAL"
        elif self.is_xml_reference:
            type_label = "XML"
        self.state_kind_label = state_kind_label or type_label
        self.child_states = {}
        self.final_outcomes = {}
        self.defaults: List[Dict[str, str]] = defaults or []
        self.xml_file: Optional[str] = None

        self._initialize_base_node_graphics(x, y)

        self._apply_default_style()

        self.title = QtWidgets.QGraphicsTextItem(self.name, self)
        self.title.setDefaultTextColor(PALETTE.text_primary)
        title_font = QtGui.QFont()
        title_font.setPointSize(10)
        title_font.setBold(True)
        self.title.setFont(title_font)

        self.type_label = QtWidgets.QGraphicsTextItem(self)
        self.type_label.setDefaultTextColor(PALETTE.text_secondary)
        type_font = QtGui.QFont()
        type_font.setPointSize(8)
        type_font.setBold(True)
        self.type_label.setFont(type_font)

        self.connection_port = ConnectionPort(self)
        self.initialize_breakpoint_marker()
        self.initialize_start_indicator()
        self.update_label()
        self.update_visual_elements()

    @property
    def start_state(self) -> Optional[str]:
        return self.model.start_state if isinstance(self.model, StateMachine) else None

    @start_state.setter
    def start_state(self, value: Optional[str]) -> None:
        if isinstance(self.model, StateMachine):
            self.model.start_state = value
            self.update_label()

    @property
    def default_outcome(self) -> Optional[str]:
        return (
            self.model.default_outcome
            if isinstance(self.model, (Concurrence, OrthogonalState))
            else None
        )

    @default_outcome.setter
    def default_outcome(self, value: Optional[str]) -> None:
        if isinstance(self.model, (Concurrence, OrthogonalState)):
            self.model.default_outcome = value
            self.update_label()

    def _apply_default_style(self) -> None:
        if self.is_xml_reference:
            self.setBrush(QtGui.QBrush(PALETTE.container_xml_fill))
            self.setPen(QtGui.QPen(PALETTE.container_xml_pen, 3))
        elif self.is_concurrence:
            self.setBrush(QtGui.QBrush(PALETTE.container_concurrence_fill))
            self.setPen(QtGui.QPen(PALETTE.container_concurrence_pen, 3))
        elif self.is_orthogonal:
            self.setBrush(QtGui.QBrush(PALETTE.container_orthogonal_fill))
            self.setPen(QtGui.QPen(PALETTE.container_orthogonal_pen, 3))
        else:
            self.setBrush(QtGui.QBrush(PALETTE.container_state_machine_fill))
            self.setPen(QtGui.QPen(PALETTE.container_state_machine_pen, 3))

    def update_start_state_label(self) -> None:
        self.update_label()

    def update_default_outcome_label(self) -> None:
        self.update_label()

    def update_label(self) -> None:
        self.title.setPlainText(self.name)
        if self.state_kind_label:
            label_text = self.state_kind_label
        elif self.is_orthogonal:
            label_text = "ORTHOGONAL"
        elif self.is_concurrence:
            label_text = "CONCURRENCE"
        else:
            label_text = "STATE MACHINE"
        self.type_label.setPlainText(label_text)
        tooltip_lines = [self.name]
        if self.description:
            tooltip_lines.append(self.description)
        if self.is_xml_reference:
            if getattr(self.model, "file_name", None):
                tooltip_lines.append(f"XML: {self.model.file_name}")
        elif self.is_orthogonal:
            tooltip_lines.append(
                f"Default: {self.default_outcome}"
                if self.default_outcome
                else "Default: (none)"
            )
        elif self.is_concurrence:
            tooltip_lines.append(
                f"Default: {self.default_outcome}"
                if self.default_outcome
                else "Default: (none)"
            )
        else:
            tooltip_lines.append(
                f"Start: {self.start_state}" if self.start_state else "Start: (none)"
            )
        self.setToolTip("\n".join(tooltip_lines))
        self.update_visual_elements()

    def update_visual_elements(self) -> None:
        self.center_text_item(self.title, -22)
        self.center_text_item(self.type_label, 6)
        self.connection_port.update_position_for_container()

    def _on_name_changed(self, value: str) -> None:
        self.update_label()

    def _on_description_changed(self, value: str) -> None:
        self.update_label()

    def _on_double_click(self, editor: Any, event: Any) -> None:
        if bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier):
            editor.enter_container(self)
        else:
            editor.edit_state()

    def _on_context_menu(self, editor: Any, event: Any) -> bool:
        if getattr(editor, "runtime_mode_enabled", False):
            if editor.show_runtime_breakpoint_menu(self, event.screenPos()):
                return True

        menu = QtWidgets.QMenu()
        enter_action = menu.addAction("Enter")
        menu.addSeparator()
        edit_action = menu.addAction(
            "View Properties" if editor.is_read_only_mode() else "Edit Properties"
        )
        delete_action = None if editor.is_read_only_mode() else menu.addAction("Delete")
        action = exec_menu(menu, event.screenPos())
        if action == enter_action:
            editor.enter_container(self)
            return True
        if action == edit_action:
            editor.edit_state()
            return True
        if action == delete_action:
            editor.delete_selected()
            return True
        return False

    def itemChange(
        self, change: QtWidgets.QGraphicsItem.GraphicsItemChange, value: Any
    ) -> Any:
        if (
            change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and isinstance(value, QtCore.QPointF)
        ):
            self.update_attached_connections()
        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedChange:
            self.update_selection_pen(
                bool(value), default_pen_callback=self._apply_default_style
            )
        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QtCore.QPointF) -> QtCore.QPointF:
        return BaseNodeMixin.get_edge_point(self, target_pos)
