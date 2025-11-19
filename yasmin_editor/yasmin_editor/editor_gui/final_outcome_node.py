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
from typing import List, Optional, Any, TYPE_CHECKING
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem, QGraphicsRectItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont

from yasmin_editor.editor_gui.connection_port import ConnectionPort

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.container_state_node import ContainerStateNode


class FinalOutcomeNode(QGraphicsRectItem):
    """Graphical representation of a final outcome."""

    def __init__(
        self, name: str, x: float, y: float, inside_container: bool = False
    ) -> None:
        super().__init__(-60, -30, 120, 60)
        self.name: str = name
        self.connections: List["ConnectionLine"] = []
        self.parent_container: Optional["ContainerStateNode"] = None
        self.inside_container: bool = inside_container

        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        self.setBrush(QBrush(QColor(255, 0, 0)))
        self.setPen(QPen(QColor(0, 0, 0), 3))

        self.text: QGraphicsTextItem = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(Qt.black)
        font: QFont = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        text_rect = self.text.boundingRect()
        self.text.setPos(-text_rect.width() / 2, -text_rect.height() / 2)

        if inside_container:
            self.connection_port: ConnectionPort = ConnectionPort(self)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            if self.parent_container:
                container_rect = self.parent_container.rect()
                outcome_rect = self.boundingRect()
                new_pos: QPointF = value

                min_x: float = container_rect.left() - outcome_rect.left() + 10
                max_x: float = container_rect.right() - outcome_rect.right() - 10
                min_y: float = container_rect.top() - outcome_rect.top() + 40
                max_y: float = container_rect.bottom() - outcome_rect.bottom() - 10

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
                self.setPen(QPen(QColor(0, 0, 0), 3))

        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine") -> None:
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine") -> None:
        if connection in self.connections:
            self.connections.remove(connection)

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the rectangle edge closest to target."""
        center: QPointF = self.scenePos()
        angle: float = math.atan2(
            target_pos.y() - center.y(), target_pos.x() - center.x()
        )
        w: float = 60
        h: float = 30
        abs_tan: float = abs(math.tan(angle)) if math.cos(angle) != 0 else float("inf")

        if abs_tan <= h / w:
            x: float = center.x() + w * (1 if math.cos(angle) > 0 else -1)
            y: float = center.y() + w * math.tan(angle) * (
                1 if math.cos(angle) > 0 else -1
            )
        else:
            y = center.y() + h * (1 if math.sin(angle) > 0 else -1)
            x = (
                center.x() + h / math.tan(angle) * (1 if math.sin(angle) > 0 else -1)
                if math.sin(angle) != 0
                else center.x()
            )
        return QPointF(x, y)
