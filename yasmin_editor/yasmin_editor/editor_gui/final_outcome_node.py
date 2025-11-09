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
from typing import List
from PyQt5.QtWidgets import (
    QGraphicsItem,
    QGraphicsTextItem,
    QGraphicsRectItem,
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont

from yasmin_editor.editor_gui.connection_line import ConnectionLine


class FinalOutcomeNode(QGraphicsRectItem):
    """Graphical representation of a final outcome."""

    def __init__(self, name: str, x: float, y: float):
        super().__init__(-60, -30, 120, 60)
        self.name = name
        self.connections: List["ConnectionLine"] = []
        self.parent_container = None  # Reference to parent container if nested

        # Set position
        self.setPos(x, y)

        # Set drag/move flags
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        # Set colors
        self.setBrush(QBrush(QColor(255, 0, 0)))  # Red
        self.setPen(QPen(QColor(0, 0, 0), 3))

        # Add text label
        self.text = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(Qt.black)
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        # Center text
        text_rect = self.text.boundingRect()
        self.text.setPos(-text_rect.width() / 2, -text_rect.height() / 2)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            # If this outcome is inside a container, constrain its movement
            if self.parent_container:
                # Get container's bounds in local coordinates
                container_rect = self.parent_container.rect()
                outcome_rect = self.boundingRect()

                # Calculate the constrained position
                new_pos = value

                # Ensure outcome stays within container bounds
                min_x = container_rect.left() - outcome_rect.left() + 10
                max_x = container_rect.right() - outcome_rect.right() - 10
                min_y = (
                    container_rect.top() - outcome_rect.top() + 40
                )  # Extra space for header
                max_y = container_rect.bottom() - outcome_rect.bottom() - 10

                # Clamp position
                constrained_x = max(min_x, min(new_pos.x(), max_x))
                constrained_y = max(min_y, min(new_pos.y(), max_y))

                value = QPointF(constrained_x, constrained_y)

            # Update all connections when the outcome moves
            for connection in self.connections:
                connection.update_position()

        return super().itemChange(change, value)

    def add_connection(self, connection: "ConnectionLine"):
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine"):
        if connection in self.connections:
            self.connections.remove(connection)

    def get_connection_point(self) -> QPointF:
        """Get the point where connections should attach."""
        return self.scenePos()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the rectangle edge closest to target."""
        center = self.scenePos()
        angle = math.atan2(target_pos.y() - center.y(), target_pos.x() - center.x())
        # Rectangle dimensions
        w = 60
        h = 30
        # Determine which edge
        abs_tan = abs(math.tan(angle)) if math.cos(angle) != 0 else float("inf")
        if abs_tan <= h / w:
            # Left or right edge
            x = center.x() + w * (1 if math.cos(angle) > 0 else -1)
            y = center.y() + w * math.tan(angle) * (1 if math.cos(angle) > 0 else -1)
        else:
            # Top or bottom edge
            y = center.y() + h * (1 if math.sin(angle) > 0 else -1)
            x = (
                center.x() + h / math.tan(angle) * (1 if math.sin(angle) > 0 else -1)
                if math.sin(angle) != 0
                else center.x()
            )
        return QPointF(x, y)
