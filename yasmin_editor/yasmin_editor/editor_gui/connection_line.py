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

"""Graphical connection item used to render transitions between states."""

import math
from typing import TYPE_CHECKING, Any, List, Tuple, Union

from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QBrush, QFont, QPainterPath, QPen
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsPathItem,
                             QGraphicsPolygonItem)

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.connection.geometry import (
    ZERO_POINT, build_arrow_polygon, compute_arrow_direction, normalize_vector,
    offset_point, use_upward_label_stack, vector_length)
from yasmin_editor.editor_gui.connection.groups import (
    get_direction_group, get_opposite_direction_group, get_self_loop_group)
from yasmin_editor.editor_gui.connection.label_items import (
    ConnectionLabelRectItem, ConnectionLabelTextItem)
from yasmin_editor.editor_gui.connection.label_layout import \
    layout_stacked_labels

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.nodes.container_state_node import \
        ContainerStateNode
    from yasmin_editor.editor_gui.nodes.final_outcome_node import \
        FinalOutcomeNode
    from yasmin_editor.editor_gui.nodes.state_node import StateNode


NodeItem = Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]


class ConnectionLine(QGraphicsPathItem):
    """Visual representation of a transition between two state nodes.

    The line is rendered as a cubic Bezier curve with an arrow head and a
    clickable outcome label. Grouped transitions share a path while their labels
    are stacked to avoid overlap.
    """

    def __init__(
        self,
        from_node: NodeItem,
        to_node: NodeItem,
        outcome: str,
    ) -> None:
        """Initialize a connection line between two nodes.

        Args:
            from_node: Source node of the transition.
            to_node: Destination node of the transition.
            outcome: Outcome name that triggers this transition.
        """
        super().__init__()
        self.from_node: NodeItem = from_node
        self.to_node: NodeItem = to_node
        self.outcome: str = outcome

        self.normal_pen = self._create_normal_pen()
        self.selected_pen = self._create_selected_pen()
        self.setPen(self.normal_pen)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setZValue(-2)

        self.arrow_head = self._create_arrow_head_item()
        self.label_bg = self._create_label_background_item()
        self.label = self._create_label_text_item(outcome)

        from_node.add_connection(self)
        to_node.add_connection(self)
        self.update_position()

    def _create_normal_pen(self) -> QPen:
        """Create the default pen used for the connection line."""
        pen = QPen(PALETTE.connection_line, 3, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        return pen

    def _create_selected_pen(self) -> QPen:
        """Create the pen used when the connection is selected."""
        pen = QPen(PALETTE.connection_selected, 4, Qt.SolidLine)
        pen.setCapStyle(Qt.RoundCap)
        return pen

    def _create_arrow_head_item(self) -> QGraphicsPolygonItem:
        """Create the arrow head graphics item."""
        arrow_head = QGraphicsPolygonItem()
        arrow_head.setBrush(QBrush(PALETTE.connection_line))
        arrow_head.setPen(QPen(PALETTE.connection_line))
        arrow_head.setZValue(-1)
        return arrow_head

    def _create_label_background_item(self) -> ConnectionLabelRectItem:
        """Create the clickable label background."""
        label_bg = ConnectionLabelRectItem(self)
        label_bg.setBrush(QBrush(PALETTE.connection_label_bg))
        label_bg.setPen(QPen(PALETTE.connection_label_pen, 1))
        label_bg.setZValue(1)
        return label_bg

    def _create_label_text_item(self, text: str) -> ConnectionLabelTextItem:
        """Create the clickable text item that displays the outcome name."""
        label = ConnectionLabelTextItem(self, text)
        label.setDefaultTextColor(PALETTE.connection_label_text)

        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        label.setFont(font)
        label.setZValue(2)
        return label

    def _get_direction_group(self) -> List["ConnectionLine"]:
        """Return all connections with the same source and target nodes."""
        return get_direction_group(self)

    def _get_opposite_direction_group(self) -> List["ConnectionLine"]:
        """Return all connections with reversed source and target nodes."""
        return get_opposite_direction_group(self)

    def _get_self_loop_group(self) -> List["ConnectionLine"]:
        """Return all self-loop connections on the source node."""
        return get_self_loop_group(self)

    def _calculate_offset(self) -> float:
        """Calculate a path offset to separate opposite connection directions."""
        if not self._get_opposite_direction_group():
            return 0.0
        return 35.0

    def _build_arrow_geometry(
        self,
        target_pos: QPointF,
        target_direction: QPointF,
    ) -> Tuple[QPointF, float]:
        """Update the arrow head polygon and return the shaft end point and angle."""
        polygon, line_end, angle = build_arrow_polygon(target_pos, target_direction)
        self.arrow_head.setPolygon(polygon)
        return line_end, angle

    def _hide_path_and_arrow(self) -> None:
        """Hide the rendered path for grouped transitions."""
        self.setPath(QPainterPath())
        self.arrow_head.setVisible(False)

    def _update_label_style(self, selected: bool | None = None) -> None:
        """Apply the current visual style based on the selection state."""
        if selected is None:
            selected = self.isSelected()

        if selected:
            self.setPen(self.selected_pen)
            self.arrow_head.setBrush(QBrush(PALETTE.connection_selected))
            self.arrow_head.setPen(QPen(PALETTE.connection_selected))
            self.label_bg.setPen(QPen(PALETTE.connection_selected, 2))
            return

        self.setPen(self.normal_pen)
        self.arrow_head.setBrush(QBrush(PALETTE.connection_line))
        self.arrow_head.setPen(QPen(PALETTE.connection_line))
        self.label_bg.setPen(QPen(PALETTE.connection_label_pen, 1))

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
                    0,
                    lambda connection=self: canvas.start_rewire_drag(connection),
                )
                return
        event.ignore()

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        """Update selection styling when the graphics item state changes."""
        if change == QGraphicsItem.ItemSelectedChange:
            self._update_label_style(bool(value))
        return super().itemChange(change, value)

    def update_position(self) -> None:
        """Recompute the path, arrow position, and label placement."""
        if self.from_node == self.to_node:
            self._update_self_loop_position()
            return

        direction_group = self._get_direction_group()
        representative = direction_group[0] if direction_group else self

        from_center = self.from_node.get_connection_point()
        to_center = self.to_node.get_connection_point()

        opposite_direction = self._get_opposite_direction_group()
        offset_amount = self._resolve_offset_amount(opposite_direction, representative)

        center_vector = QPointF(
            to_center.x() - from_center.x(),
            to_center.y() - from_center.y(),
        )
        center_direction = normalize_vector(center_vector)
        if vector_length(center_direction) <= 1e-9:
            center_direction = QPointF(1.0, 0.0)

        normal_direction = QPointF(-center_direction.y(), center_direction.x())
        anchor_offset = offset_point(ZERO_POINT, normal_direction, offset_amount)
        from_pos, to_pos = self._compute_edge_points(
            from_center,
            to_center,
            anchor_offset,
        )
        source_direction, target_direction = self._compute_radial_directions(
            from_center,
            from_pos,
            to_center,
            to_pos,
            center_direction,
        )

        ctrl1, ctrl2 = self._compute_control_points(
            from_pos,
            to_pos,
            source_direction,
            target_direction,
            normal_direction,
            offset_amount,
        )
        shaft_end = self._update_arrow_head(from_center, to_center, to_pos, ctrl2)
        path = self._build_curve_path(from_pos, ctrl1, ctrl2, shaft_end)

        if self == representative:
            self.setPath(path)
            self.arrow_head.setVisible(True)
        else:
            self._hide_path_and_arrow()

        self._update_label_positions(
            path,
            direction_group,
            opposite_direction,
            from_center,
            to_center,
        )
        self._update_label_style()

    def _resolve_offset_amount(
        self,
        opposite_direction: List["ConnectionLine"],
        representative: "ConnectionLine",
    ) -> float:
        """Resolve the lateral offset used for parallel opposite directions."""
        offset_amount = self._calculate_offset()
        if not opposite_direction:
            return offset_amount

        opposite_representative = opposite_direction[0]
        if representative.from_node == opposite_representative.to_node:
            return 35.0
        return -35.0

    def _compute_edge_points(
        self,
        from_center: QPointF,
        to_center: QPointF,
        anchor_offset: QPointF,
    ) -> Tuple[QPointF, QPointF]:
        """Compute the path start and end points on the node boundaries."""
        from_pos = self.from_node.get_edge_point(
            QPointF(
                to_center.x() + anchor_offset.x(),
                to_center.y() + anchor_offset.y(),
            )
        )
        to_pos = self.to_node.get_edge_point(
            QPointF(
                from_center.x() + anchor_offset.x(),
                from_center.y() + anchor_offset.y(),
            )
        )
        return from_pos, to_pos

    def _compute_radial_directions(
        self,
        from_center: QPointF,
        from_pos: QPointF,
        to_center: QPointF,
        to_pos: QPointF,
        center_direction: QPointF,
    ) -> Tuple[QPointF, QPointF]:
        """Compute the tangent directions at the start and end of the path."""
        source_direction = normalize_vector(
            QPointF(from_pos.x() - from_center.x(), from_pos.y() - from_center.y())
        )
        target_direction = normalize_vector(
            QPointF(to_center.x() - to_pos.x(), to_center.y() - to_pos.y())
        )

        if vector_length(source_direction) <= 1e-9:
            source_direction = center_direction
        if vector_length(target_direction) <= 1e-9:
            target_direction = center_direction
        return source_direction, target_direction

    def _compute_control_points(
        self,
        from_pos: QPointF,
        to_pos: QPointF,
        source_direction: QPointF,
        target_direction: QPointF,
        normal_direction: QPointF,
        offset_amount: float,
    ) -> Tuple[QPointF, QPointF]:
        """Compute the cubic Bezier control points for the transition path."""
        path_length = vector_length(
            QPointF(to_pos.x() - from_pos.x(), to_pos.y() - from_pos.y())
        )
        tangent_length = max(36.0, min(90.0, path_length * 0.35))
        offset_vector = QPointF(
            normal_direction.x() * offset_amount,
            normal_direction.y() * offset_amount,
        )

        ctrl1 = QPointF(
            from_pos.x() + source_direction.x() * tangent_length + offset_vector.x(),
            from_pos.y() + source_direction.y() * tangent_length + offset_vector.y(),
        )
        ctrl2 = QPointF(
            to_pos.x() - target_direction.x() * tangent_length + offset_vector.x(),
            to_pos.y() - target_direction.y() * tangent_length + offset_vector.y(),
        )
        return ctrl1, ctrl2

    def _update_arrow_head(
        self,
        from_center: QPointF,
        to_center: QPointF,
        to_pos: QPointF,
        ctrl2: QPointF,
    ) -> QPointF:
        """Update the arrow head and return the path end point before the arrow."""
        target_direction = compute_arrow_direction(
            to_pos,
            ctrl2,
            QPointF(to_center.x() - from_center.x(), to_center.y() - from_center.y()),
        )
        line_end, _ = self._build_arrow_geometry(to_pos, target_direction)
        return line_end

    def _build_curve_path(
        self,
        from_pos: QPointF,
        ctrl1: QPointF,
        ctrl2: QPointF,
        line_end: QPointF,
    ) -> QPainterPath:
        """Build the cubic Bezier path used for the rendered connection."""
        path = QPainterPath()
        path.moveTo(from_pos)
        path.cubicTo(ctrl1, ctrl2, line_end)
        return path

    def _update_label_positions(
        self,
        path: QPainterPath,
        direction_group: List["ConnectionLine"],
        opposite_direction: List["ConnectionLine"],
        from_center: QPointF,
        to_center: QPointF,
    ) -> None:
        """Update label positions for grouped and opposing transitions."""
        label_anchor = QPointF(path.pointAtPercent(0.5))
        label_stack_direction = "center"

        if opposite_direction:
            label_gap = 8.0
            if use_upward_label_stack(from_center, to_center):
                label_anchor.setY(label_anchor.y() - label_gap)
                label_stack_direction = "up"
            else:
                label_anchor.setY(label_anchor.y() + label_gap)
                label_stack_direction = "down"

        layout_stacked_labels(direction_group, label_anchor, label_stack_direction)

    def _update_self_loop_position(self) -> None:
        """Update position for a self-loop transition.

        Self-loops are rendered as an arc above the node. Multiple self-loops on
        the same node share the path while their labels are stacked.
        """
        node = self.from_node
        bounds = node.sceneBoundingRect()
        center_x = bounds.center().x()
        node_top = bounds.top() + 4.0
        node_width = bounds.width()

        self_loops = self._get_self_loop_group()
        representative = self_loops[0] if self_loops else self

        loop_height = max(72.0, bounds.height() + 16.0)
        loop_width = max(60.0, node_width * 0.45)
        anchor_half_width = max(16.0, min(24.0, node_width * 0.16))

        start_x = center_x - anchor_half_width
        end_x = center_x + anchor_half_width
        start_pos = QPointF(start_x, node_top)
        end_pos = QPointF(end_x, node_top)

        ctrl1 = QPointF(start_x - loop_width / 2.0, node_top - loop_height)
        ctrl2 = QPointF(end_x + loop_width / 2.0, node_top - loop_height)

        arrow_angle = math.pi / 2.0 + math.radians(16.0)
        target_direction = QPointF(math.cos(arrow_angle), math.sin(arrow_angle))
        line_end, _ = self._build_arrow_geometry(end_pos, target_direction)

        path = QPainterPath()
        path.moveTo(start_pos)
        path.cubicTo(ctrl1, ctrl2, line_end)

        if self == representative:
            self.setPath(path)
            self.arrow_head.setVisible(True)
        else:
            self._hide_path_and_arrow()

        label_anchor = QPointF(center_x, node_top - loop_height + 8.0)
        layout_stacked_labels(self_loops, label_anchor)
        self._update_label_style()
