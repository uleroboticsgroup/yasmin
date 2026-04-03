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

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Union

from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QBrush, QFont, QPen
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsRectItem, QGraphicsTextItem, QMenu

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection_port import ConnectionPort
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine

if TYPE_CHECKING:
    from yasmin_plugins_manager.plugin_info import PluginInfo


class ContainerStateNode(QGraphicsRectItem, BaseNodeMixin):
    """Graphical representation of a nested State Machine or Concurrence."""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        is_concurrence: bool = False,
        remappings: Optional[Dict[str, str]] = None,
        outcomes: Optional[List[str]] = None,
        start_state: Optional[str] = None,
        default_outcome: Optional[str] = None,
        description: str = "",
        defaults: Optional[List[Dict[str, str]]] = None,
        model: Optional[Union[StateMachine, Concurrence, State]] = None,
        state_kind_label: Optional[str] = None,
        is_xml_reference: bool = False,
    ) -> None:
        super().__init__(-65, -40, 130, 80)
        self.model: Union[StateMachine, Concurrence, State] = model or (
            Concurrence(
                name=name,
                description=description,
                default_outcome=default_outcome,
                remappings=dict(remappings or {}),
                outcomes=[Outcome(name=item) for item in outcomes or []],
            )
            if is_concurrence
            else StateMachine(
                name=name,
                description=description,
                start_state=start_state,
                remappings=dict(remappings or {}),
                outcomes=[Outcome(name=item) for item in outcomes or []],
            )
        )
        self.plugin_info: Optional["PluginInfo"] = None
        self.is_state_machine = isinstance(self.model, StateMachine)
        self.is_concurrence = isinstance(self.model, Concurrence)
        self.is_xml_reference = bool(is_xml_reference)
        self.state_kind_label = state_kind_label or (
            "XML" if self.is_xml_reference else None
        )
        self.child_states = {}
        self.final_outcomes = {}
        self.defaults: List[Dict[str, str]] = defaults or []
        self.xml_file: Optional[str] = None

        self._initialize_base_node_graphics(x, y)

        self._apply_default_style()

        self.title = QGraphicsTextItem(self.name, self)
        self.title.setDefaultTextColor(PALETTE.text_primary)
        title_font = QFont()
        title_font.setPointSize(10)
        title_font.setBold(True)
        self.title.setFont(title_font)

        self.type_label = QGraphicsTextItem(self)
        self.type_label.setDefaultTextColor(PALETTE.text_secondary)
        type_font = QFont()
        type_font.setPointSize(8)
        type_font.setBold(True)
        self.type_label.setFont(type_font)

        self.connection_port = ConnectionPort(self)
        self.update_label()
        self.update_visual_elements()

    @property
    def name(self) -> str:
        return self.model.name

    @name.setter
    def name(self, value: str) -> None:
        self.model.name = value
        self.update_label()

    @property
    def description(self) -> str:
        return self.model.description

    @description.setter
    def description(self, value: str) -> None:
        self.model.description = value
        self.update_label()

    @property
    def remappings(self) -> Dict[str, str]:
        return self.model.remappings

    @remappings.setter
    def remappings(self, value: Dict[str, str]) -> None:
        self.model.remappings.clear()
        self.model.remappings.update(value or {})

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
        return self.model.default_outcome if isinstance(self.model, Concurrence) else None

    @default_outcome.setter
    def default_outcome(self, value: Optional[str]) -> None:
        if isinstance(self.model, Concurrence):
            self.model.default_outcome = value
            self.update_label()

    def _apply_default_style(self) -> None:
        if self.is_xml_reference:
            self.setBrush(QBrush(PALETTE.container_xml_fill))
            self.setPen(QPen(PALETTE.container_xml_pen, 3))
        elif self.is_concurrence:
            self.setBrush(QBrush(PALETTE.container_concurrence_fill))
            self.setPen(QPen(PALETTE.container_concurrence_pen, 3))
        else:
            self.setBrush(QBrush(PALETTE.container_state_machine_fill))
            self.setPen(QPen(PALETTE.container_state_machine_pen, 3))

    def update_start_state_label(self) -> None:
        self.update_label()

    def update_default_outcome_label(self) -> None:
        self.update_label()

    def update_label(self) -> None:
        self.title.setPlainText(self.name)
        self.type_label.setPlainText(
            self.state_kind_label
            if self.state_kind_label
            else ("CONCURRENCE" if self.is_concurrence else "STATE MACHINE")
        )
        tooltip_lines = [self.name]
        if self.description:
            tooltip_lines.append(self.description)
        if self.is_xml_reference:
            if getattr(self.model, "file_name", None):
                tooltip_lines.append(f"XML: {self.model.file_name}")
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

    @property
    def parameter_mappings(self) -> Dict[str, str]:
        return self.model.parameter_mappings

    @parameter_mappings.setter
    def parameter_mappings(self, value: Dict[str, str]) -> None:
        self.model.parameter_mappings.clear()
        self.model.parameter_mappings.update(value or {})

    def mouseDoubleClickEvent(self, event: Any) -> None:
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                if bool(event.modifiers() & Qt.ControlModifier):
                    canvas.editor_ref.enter_container(self)
                else:
                    canvas.editor_ref.edit_state()
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def contextMenuEvent(self, event: Any) -> None:
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                menu = QMenu()
                enter_action = menu.addAction("Enter")
                menu.addSeparator()
                edit_action = menu.addAction("Edit Properties")
                delete_action = menu.addAction("Delete")
                action = menu.exec_(event.screenPos())
                if action == enter_action:
                    self.setSelected(True)
                    canvas.editor_ref.enter_container(self)
                    event.accept()
                    return
                if action == edit_action:
                    self.setSelected(True)
                    canvas.editor_ref.edit_state()
                    event.accept()
                    return
                if action == delete_action:
                    self.setSelected(True)
                    canvas.editor_ref.delete_selected()
                    event.accept()
                    return
        super().contextMenuEvent(event)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            self.update_attached_connections()
        elif change == QGraphicsItem.ItemSelectedChange:
            self.update_selection_pen(
                bool(value), default_pen_callback=self._apply_default_style
            )
        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        return BaseNodeMixin.get_edge_point(self, target_pos)
