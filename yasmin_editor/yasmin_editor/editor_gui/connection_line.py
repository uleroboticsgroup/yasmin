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
from typing import TYPE_CHECKING, Any, List, Union

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QBrush, QFont, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsPathItem,
                             QGraphicsPolygonItem, QGraphicsRectItem,
                             QGraphicsTextItem)
from yasmin_editor.editor_gui.colors import PALETTE

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.container_state_node import \
        ContainerStateNode
    from yasmin_editor.editor_gui.final_outcome_node import FinalOutcomeNode
    from yasmin_editor.editor_gui.state_node import StateNode


class _ConnectionLabelRectItem(QGraphicsRectItem):
    """Clickable label background that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine") -> None:
        super().__init__()
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)


class _ConnectionLabelTextItem(QGraphicsTextItem):
    """Clickable label text that selects the owning connection."""

    def __init__(self, owner: "ConnectionLine", text: str) -> None:
        super().__init__(text)
        self.owner = owner
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event: Any) -> None:
        self.owner.select_from_label(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        self.owner.start_rewire_from_label(event)


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

        pen: QPen = QPen(PALETTE.connection_line, 3, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        self.setPen(pen)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        self.normal_pen: QPen = pen
        self.selected_pen: QPen = QPen(PALETTE.connection_selected, 4, Qt.SolidLine)
        self.selected_pen.setCapStyle(Qt.RoundCap)

        self.setZValue(-2)

        self.arrow_head: QGraphicsPolygonItem = QGraphicsPolygonItem()
        self.arrow_head.setBrush(QBrush(PALETTE.connection_line))
        self.arrow_head.setPen(QPen(PALETTE.connection_line))
        self.arrow_head.setZValue(-2)

        self.label_bg: QGraphicsRectItem = _ConnectionLabelRectItem(self)
        self.label_bg.setBrush(QBrush(PALETTE.connection_label_bg))
        self.label_bg.setPen(QPen(PALETTE.connection_label_pen, 1))
        self.label_bg.setZValue(1)

        self.label: QGraphicsTextItem = _ConnectionLabelTextItem(self, outcome)
        self.label.setDefaultTextColor(PALETTE.connection_label_text)
        font: QFont = QFont()
        font.setPointSize(9)
        font.setBold(True)
        self.label.setFont(font)
        self.label.setZValue(2)

        from_node.add_connection(self)
        to_node.add_connection(self)
        self.update_position()

    def _get_direction_group(self) -> List["ConnectionLine"]:
        """Get all connections with the same source and target nodes."""
        group: List["ConnectionLine"] = []
        for connection in list(self.from_node.connections) + list(
            self.to_node.connections
        ):
            if (
                connection.from_node == self.from_node
                and connection.to_node == self.to_node
                and connection not in group
            ):
                group.append(connection)
        group.sort(key=lambda connection: connection.outcome)
        return group

    def _get_opposite_direction_group(self) -> List["ConnectionLine"]:
        """Get all connections with the reverse source and target nodes."""
        group: List["ConnectionLine"] = []
        for connection in list(self.from_node.connections) + list(
            self.to_node.connections
        ):
            if (
                connection.from_node == self.to_node
                and connection.to_node == self.from_node
                and connection not in group
            ):
                group.append(connection)
        group.sort(key=lambda connection: connection.outcome)
        return group

    def _get_self_loop_group(self) -> List["ConnectionLine"]:
        """Get all self-loop connections on this node."""
        group: List["ConnectionLine"] = []
        for connection in self.from_node.connections:
            if (
                connection.from_node == connection.to_node
                and connection.from_node == self.from_node
                and connection not in group
            ):
                group.append(connection)
        group.sort(key=lambda connection: connection.outcome)
        return group

    def _calculate_offset(self) -> float:
        """Calculate offset for this connection to avoid overlap with other connections.

        Computes a perpendicular offset based on the number and direction of
        connections between the same pair of nodes to prevent visual overlap.

        Returns:
            The offset amount in pixels (positive or negative).
        """
        opposite_direction: List["ConnectionLine"] = (
            self._get_opposite_direction_group()
        )
        if not opposite_direction:
            return 0.0
        return 35.0

    def _set_arrow_polygon(self, target_pos: QPointF, angle: float) -> None:
        """Update the arrow head polygon for the current end point and angle."""
        arrow_size: float = 12
        arrow_p1: QPointF = target_pos - QPointF(
            math.cos(angle - math.pi / 6) * arrow_size,
            math.sin(angle - math.pi / 6) * arrow_size,
        )
        arrow_p2: QPointF = target_pos - QPointF(
            math.cos(angle + math.pi / 6) * arrow_size,
            math.sin(angle + math.pi / 6) * arrow_size,
        )
        self.arrow_head.setPolygon(QPolygonF([target_pos, arrow_p1, arrow_p2]))

    def _hide_path_and_arrow(self) -> None:
        """Hide the rendered path for grouped transitions."""
        self.setPath(QPainterPath())
        self.arrow_head.setVisible(False)

    def _update_label_style(self, selected: bool | None = None) -> None:
        """Apply the visual style of the label based on the selection state."""
        if selected is None:
            selected = self.isSelected()
        if selected:
            self.setPen(self.selected_pen)
            self.arrow_head.setBrush(QBrush(PALETTE.connection_selected))
            self.arrow_head.setPen(QPen(PALETTE.connection_selected))
            self.label_bg.setPen(QPen(PALETTE.connection_selected, 2))
        else:
            self.setPen(self.normal_pen)
            self.arrow_head.setBrush(QBrush(PALETTE.connection_line))
            self.arrow_head.setPen(QPen(PALETTE.connection_line))
            self.label_bg.setPen(QPen(PALETTE.connection_label_pen, 1))

    def _layout_stacked_labels(
        self,
        group: List["ConnectionLine"],
        anchor_point: QPointF,
    ) -> None:
        """Layout the labels of a grouped connection as a vertical stack."""
        if not group:
            return

        padding: float = 4.0
        spacing: float = 2.0
        box_heights: List[float] = []
        total_height: float = 0.0

        for connection in group:
            label_rect = connection.label.boundingRect()
            box_height = label_rect.height() + padding * 2
            box_heights.append(box_height)
            total_height += box_height

        total_height += spacing * max(0, len(group) - 1)
        current_top = anchor_point.y() - total_height / 2.0

        for connection, box_height in zip(group, box_heights):
            label_rect = connection.label.boundingRect()
            box_width = label_rect.width() + padding * 2
            box_left = anchor_point.x() - box_width / 2.0
            connection.label_bg.setRect(
                QRectF(box_left, current_top, box_width, box_height)
            )
            connection.label.setPos(box_left + padding, current_top + padding)
            current_top += box_height + spacing

    def select_from_label(self, event: Any) -> None:
        """Select the connection when the label or label background is clicked."""
        scene = self.scene()
        if scene is not None and not bool(event.modifiers() & Qt.ControlModifier):
            scene.clearSelection()
        self.setSelected(True)
        event.accept()

    def start_rewire_from_label(self, event: Any) -> None:
        """Start rewiring mode when the transition label is double-clicked."""
        self.select_from_label(event)
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "start_rewire_drag"):
                QTimer.singleShot(
                    0, lambda connection=self: canvas.start_rewire_drag(connection)
                )
                return
        event.ignore()

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        """Handle item changes like selection state.

        Args:
            change: The type of change occurring.
            value: The new value for the change.

        Returns:
            The potentially modified value.
        """
        if change == QGraphicsItem.ItemSelectedChange:
            self._update_label_style(bool(value))
        return super().itemChange(change, value)

    def update_position(self) -> None:
        """Update the connection line with a smooth curved path.

        Recalculates the cubic Bezier curve path between nodes, including
        offset for multiple connections, arrow head position, and label placement.
        For self-loops, draws a loop arc above the node.
        """
        # Check if this is a self-loop
        if self.from_node == self.to_node:
            self._update_self_loop_position()
            return

        direction_group = self._get_direction_group()
        representative = direction_group[0] if direction_group else self

        from_center: QPointF = self.from_node.get_connection_point()
        to_center: QPointF = self.to_node.get_connection_point()
        from_pos: QPointF = self.from_node.get_edge_point(to_center)
        to_pos: QPointF = self.to_node.get_edge_point(from_center)

        opposite_direction: List["ConnectionLine"] = (
            self._get_opposite_direction_group()
        )
        offset_amount: float = self._calculate_offset()
        if opposite_direction:
            opposite_representative = opposite_direction[0]
            if representative.from_node == opposite_representative.to_node:
                offset_amount = 35.0
            else:
                offset_amount = -35.0

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

        if self == representative:
            self.setPath(path)
            tangent_point: QPointF = path.pointAtPercent(0.95)
            angle = math.atan2(
                to_pos.y() - tangent_point.y(),
                to_pos.x() - tangent_point.x(),
            )
            self._set_arrow_polygon(to_pos, angle)
            self.arrow_head.setVisible(True)
        else:
            self._hide_path_and_arrow()

        mid_point: QPointF = path.pointAtPercent(0.5)
        self._layout_stacked_labels(direction_group, mid_point)
        self._update_label_style()

    def _update_self_loop_position(self) -> None:
        """Update position for a self-loop (transition from a state to itself).

        Draws a loop arc above the node with an arrow pointing back into the node.
        Multiple self-loops on the same node are offset to avoid overlap.
        """
        node = self.from_node
        bounds = node.sceneBoundingRect()
        center_x: float = bounds.center().x()
        node_top: float = bounds.top() + 4.0
        node_width: float = bounds.width()

        self_loops = self._get_self_loop_group()
        representative = self_loops[0] if self_loops else self

        loop_height: float = max(72.0, bounds.height() + 16.0)
        loop_width: float = max(60.0, node_width * 0.45)
        anchor_half_width: float = max(16.0, min(24.0, node_width * 0.16))

        start_x = center_x - anchor_half_width
        end_x = center_x + anchor_half_width

        start_pos = QPointF(start_x, node_top)
        end_pos = QPointF(end_x, node_top)

        ctrl1 = QPointF(start_x - loop_width / 2.0, node_top - loop_height)
        ctrl2 = QPointF(end_x + loop_width / 2.0, node_top - loop_height)

        path: QPainterPath = QPainterPath()
        path.moveTo(start_pos)
        path.cubicTo(ctrl1, ctrl2, end_pos)

        if self == representative:
            self.setPath(path)
            self._set_arrow_polygon(end_pos, math.pi / 2.0)
            self.arrow_head.setVisible(True)
        else:
            self._hide_path_and_arrow()

        label_anchor = QPointF(center_x, node_top - loop_height + 8.0)
        self._layout_stacked_labels(self_loops, label_anchor)
        self._update_label_style()
