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

import math
from typing import Dict, List, Set, Optional, Any, TYPE_CHECKING
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsEllipseItem, QGraphicsTextItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont
from yasmin_editor.plugins_manager.plugin_info import PluginInfo

from yasmin_editor.editor_gui.connection_port import ConnectionPort

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.container_state_node import ContainerStateNode


class StateNode(QGraphicsEllipseItem):
    """Graphical representation of a regular state (not container)."""

    def __init__(
        self,
        name: str,
        plugin_info: PluginInfo,
        x: float,
        y: float,
        remappings: Optional[Dict[str, str]] = None,
    ) -> None:
        super().__init__(-60, -40, 120, 80)
        self.name: str = name
        self.plugin_info: PluginInfo = plugin_info
        self.is_state_machine: bool = False
        self.is_concurrence: bool = False
        self.connections: List["ConnectionLine"] = []
        self.remappings: Dict[str, str] = remappings or {}
        self.parent_container: Optional["ContainerStateNode"] = None

        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        if plugin_info and plugin_info.plugin_type == "python":
            self.setBrush(QBrush(QColor(144, 238, 144)))
        elif plugin_info and plugin_info.plugin_type == "cpp":
            self.setBrush(QBrush(QColor(255, 182, 193)))
        else:
            self.setBrush(QBrush(QColor(255, 165, 0)))

        self.setPen(QPen(QColor(0, 0, 0), 2))

        self.text: QGraphicsTextItem = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(Qt.black)
        font: QFont = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        text_rect = self.text.boundingRect()
        self.text.setPos(-text_rect.width() / 2, -text_rect.height() / 2)

        if plugin_info:
            type_text: str = plugin_info.plugin_type.upper()
            self.type_label: QGraphicsTextItem = QGraphicsTextItem(type_text, self)
            self.type_label.setDefaultTextColor(Qt.darkGray)
            type_font: QFont = QFont()
            type_font.setPointSize(8)
            self.type_label.setFont(type_font)
            type_rect = self.type_label.boundingRect()
            self.type_label.setPos(-type_rect.width() / 2, 10)

        self.connection_port: ConnectionPort = ConnectionPort(self)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Handle double-click to edit state."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                canvas.editor_ref.edit_state()
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            if self.parent_container:
                container_rect = self.parent_container.rect()
                state_rect = self.boundingRect()
                new_pos: QPointF = value

                min_x: float = container_rect.left() - state_rect.left() + 10
                max_x: float = container_rect.right() - state_rect.right() - 10
                min_y: float = container_rect.top() - state_rect.top() + 40
                max_y: float = container_rect.bottom() - state_rect.bottom() - 10

                constrained_x: float = max(min_x, min(new_pos.x(), max_x))
                constrained_y: float = max(min_y, min(new_pos.y(), max_y))
                value = QPointF(constrained_x, constrained_y)

            for connection in self.connections:
                connection.update_position()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            if self.parent_container:
                self.parent_container.auto_resize_for_children()

        elif change == QGraphicsItem.ItemSelectedChange:
            if value:
                self.setPen(QPen(QColor(255, 200, 0), 4))
            else:
                self.setPen(QPen(QColor(0, 0, 180), 3))

        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine") -> None:
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine") -> None:
        if connection in self.connections:
            self.connections.remove(connection)

    def get_used_outcomes(self) -> Set[str]:
        """Get set of outcomes that already have connections."""
        return {
            connection.from_node.name + connection.outcome
            for connection in self.connections
        }

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach (center of port)."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the ellipse edge closest to target."""
        center: QPointF = self.scenePos()
        angle: float = math.atan2(
            target_pos.y() - center.y(), target_pos.x() - center.x()
        )
        rx: float = 60
        ry: float = 40
        x: float = center.x() + rx * math.cos(angle)
        y: float = center.y() + ry * math.sin(angle)
        return QPointF(x, y)
