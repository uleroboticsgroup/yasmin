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
from typing import Union, List, Any, TYPE_CHECKING
from PyQt5.QtWidgets import (
    QGraphicsItem,
    QGraphicsTextItem,
    QGraphicsRectItem,
    QGraphicsPolygonItem,
    QGraphicsPathItem,
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont, QPolygonF, QPainterPath

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.state_node import StateNode
    from yasmin_editor.editor_gui.container_state_node import ContainerStateNode
    from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode


class ConnectionLine(QGraphicsPathItem):
    """Graphical representation of a transition between states with curved line.

    Provides a visual representation of state transitions using Bezier curves,
    with automatic routing to avoid overlaps and visual feedback for selection.
    """

    def __init__(
        self,
        from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        to_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"],
        outcome: str,
    ) -> None:
        """Initialize a connection line between two nodes.

        Args:
            from_node: The source state node.
            to_node: The destination state node.
            outcome: The outcome name that triggers this transition.
        """
        super().__init__()
        self.from_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"] = (
            from_node
        )
        self.to_node: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"] = (
            to_node
        )
        self.outcome: str = outcome

        pen: QPen = QPen(QColor(60, 60, 180), 3, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        self.setPen(pen)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        self.normal_pen: QPen = pen
        self.selected_pen: QPen = QPen(QColor(255, 100, 0), 4, Qt.SolidLine)
        self.selected_pen.setCapStyle(Qt.RoundCap)

        self.arrow_head: QGraphicsPolygonItem = QGraphicsPolygonItem()
        self.arrow_head.setBrush(QBrush(QColor(60, 60, 180)))
        self.arrow_head.setPen(QPen(QColor(60, 60, 180)))

        self.label_bg: QGraphicsRectItem = QGraphicsRectItem()
        self.label_bg.setBrush(QBrush(QColor(255, 255, 255, 230)))
        self.label_bg.setPen(QPen(QColor(60, 60, 180), 1))

        self.label: QGraphicsTextItem = QGraphicsTextItem(outcome)
        self.label.setDefaultTextColor(QColor(0, 0, 100))
        font: QFont = QFont()
        font.setPointSize(9)
        font.setBold(True)
        self.label.setFont(font)

        self.update_position()

        from_node.add_connection(self)
        to_node.add_connection(self)

    def _calculate_offset(self) -> float:
        """Calculate offset for this connection to avoid overlap with other connections.

        Computes a perpendicular offset based on the number and direction of
        connections between the same pair of nodes to prevent visual overlap.

        Returns:
            The offset amount in pixels (positive or negative).
        """
        same_direction: List["ConnectionLine"] = []
        opposite_direction: List["ConnectionLine"] = []

        all_connections: List["ConnectionLine"] = list(self.from_node.connections) + list(
            self.to_node.connections
        )

        for conn in all_connections:
            if conn.from_node == self.from_node and conn.to_node == self.to_node:
                if conn not in same_direction:
                    same_direction.append(conn)
            elif conn.from_node == self.to_node and conn.to_node == self.from_node:
                if conn not in opposite_direction:
                    opposite_direction.append(conn)

        is_same_direction: bool = self in same_direction
        connections_in_my_direction: List["ConnectionLine"] = (
            same_direction if is_same_direction else opposite_direction
        )
        connections_in_other_direction: List["ConnectionLine"] = (
            opposite_direction if is_same_direction else same_direction
        )

        if len(connections_in_my_direction) <= 1:
            if len(connections_in_other_direction) > 0:
                return 35 if is_same_direction else -35
            else:
                return 0

        connections_in_my_direction.sort(key=lambda c: c.outcome)

        try:
            my_index: int = connections_in_my_direction.index(self)
        except ValueError:
            return 0

        num_in_direction: int = len(connections_in_my_direction)
        has_opposite: bool = len(connections_in_other_direction) > 0

        base_offset: float = (
            35 if (is_same_direction and has_opposite) else (-35 if has_opposite else 0)
        )
        spacing: float = 40

        if num_in_direction == 2:
            offset: float = spacing * (0.5 if my_index == 0 else -0.5)
        else:
            center_index: float = (num_in_direction - 1) / 2
            offset_from_center: float = my_index - center_index
            offset = offset_from_center * spacing * 0.7

        return base_offset + offset

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        """Handle item changes like selection state.

        Args:
            change: The type of change occurring.
            value: The new value for the change.

        Returns:
            The potentially modified value.
        """
        if change == QGraphicsItem.ItemSelectedChange:
            if value:
                self.setPen(self.selected_pen)
                self.arrow_head.setBrush(QBrush(QColor(255, 100, 0)))
                self.arrow_head.setPen(QPen(QColor(255, 100, 0)))
            else:
                self.setPen(self.normal_pen)
                self.arrow_head.setBrush(QBrush(QColor(60, 60, 180)))
                self.arrow_head.setPen(QPen(QColor(60, 60, 180)))
        return super().itemChange(change, value)

    def update_position(self) -> None:
        """Update the connection line with a smooth curved path.

        Recalculates the cubic Bezier curve path between nodes, including
        offset for multiple connections, arrow head position, and label placement.
        """
        from_center: QPointF = self.from_node.get_connection_point()
        to_center: QPointF = self.to_node.get_connection_point()
        from_pos: QPointF = self.from_node.get_edge_point(to_center)
        to_pos: QPointF = self.to_node.get_edge_point(from_center)

        offset_amount: float = self._calculate_offset()

        path: QPainterPath = QPainterPath()
        path.moveTo(from_pos)

        dx: float = to_pos.x() - from_pos.x()
        dy: float = to_pos.y() - from_pos.y()

        angle: float = math.atan2(dy, dx)
        perp_angle: float = angle + math.pi / 2

        ctrl1: QPointF = QPointF(
            from_pos.x() + dx * 0.25 + math.cos(perp_angle) * offset_amount,
            from_pos.y() + dy * 0.25 + math.sin(perp_angle) * offset_amount,
        )
        ctrl2: QPointF = QPointF(
            from_pos.x() + dx * 0.75 + math.cos(perp_angle) * offset_amount,
            from_pos.y() + dy * 0.75 + math.sin(perp_angle) * offset_amount,
        )

        path.cubicTo(ctrl1, ctrl2, to_pos)
        self.setPath(path)

        t: float = 0.95
        tangent_point: QPointF = path.pointAtPercent(t)
        angle = math.atan2(to_pos.y() - tangent_point.y(), to_pos.x() - tangent_point.x())

        arrow_size: float = 12
        arrow_p1: QPointF = to_pos - QPointF(
            math.cos(angle - math.pi / 6) * arrow_size,
            math.sin(angle - math.pi / 6) * arrow_size,
        )
        arrow_p2: QPointF = to_pos - QPointF(
            math.cos(angle + math.pi / 6) * arrow_size,
            math.sin(angle + math.pi / 6) * arrow_size,
        )

        arrow_polygon: QPolygonF = QPolygonF([to_pos, arrow_p1, arrow_p2])
        self.arrow_head.setPolygon(arrow_polygon)

        mid_point: QPointF = path.pointAtPercent(0.5)
        label_rect = self.label.boundingRect()
        padding: float = 4

        self.label_bg.setRect(
            mid_point.x() - label_rect.width() / 2 - padding,
            mid_point.y() - label_rect.height() / 2 - padding,
            label_rect.width() + padding * 2,
            label_rect.height() + padding * 2,
        )
        self.label.setPos(
            mid_point.x() - label_rect.width() / 2,
            mid_point.y() - label_rect.height() / 2,
        )
