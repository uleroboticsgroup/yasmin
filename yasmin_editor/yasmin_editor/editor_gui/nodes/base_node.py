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

import math
from typing import TYPE_CHECKING, Any, Callable, List, Optional, Set

from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QBrush, QColor, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPolygonItem,
    QGraphicsTextItem,
)
from yasmin_editor.editor_gui.colors import PALETTE

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine
    from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode


class BaseNodeMixin:
    """Reusable functionality shared by graphical editor nodes."""

    def _initialize_base_node_graphics(self, x: float, y: float) -> None:
        self.connections: List["ConnectionLine"] = []
        self.parent_container: Optional["ContainerStateNode"] = None

        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

    def add_connection(self, connection: "ConnectionLine") -> None:
        if connection not in self.connections:
            self.connections.append(connection)

    def remove_connection(self, connection: "ConnectionLine") -> None:
        if connection in self.connections:
            self.connections.remove(connection)

    def get_used_outcomes(self) -> Set[str]:
        return {
            connection.outcome
            for connection in self.connections
            if connection.from_node == self
        }

    def center_text_item(self, item: QGraphicsTextItem, y: float) -> None:
        text_rect = item.boundingRect()
        item.setPos(-text_rect.width() / 2.0, y)

    def get_connection_point(self) -> QPointF:
        return self.sceneBoundingRect().center()

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        bounds = self.sceneBoundingRect()
        center = bounds.center()
        dx = target_pos.x() - center.x()
        dy = target_pos.y() - center.y()

        if math.isclose(dx, 0.0, abs_tol=1e-9) and math.isclose(dy, 0.0, abs_tol=1e-9):
            return center

        half_width = max(bounds.width() / 2.0, 1.0)
        half_height = max(bounds.height() / 2.0, 1.0)
        scale = 1.0 / max(abs(dx) / half_width, abs(dy) / half_height, 1e-9)
        return QPointF(center.x() + dx * scale, center.y() + dy * scale)

    def constrain_position_to_parent(
        self,
        value: QPointF,
        top_margin: float = 40.0,
        side_margin: float = 10.0,
        bottom_margin: float = 10.0,
    ) -> QPointF:
        if not self.parent_container:
            return value

        container_rect = self.parent_container.rect()
        node_rect = self.boundingRect()

        min_x = container_rect.left() - node_rect.left() + side_margin
        max_x = container_rect.right() - node_rect.right() - side_margin
        min_y = container_rect.top() - node_rect.top() + top_margin
        max_y = container_rect.bottom() - node_rect.bottom() - bottom_margin

        constrained_x = max(min_x, min(value.x(), max_x))
        constrained_y = max(min_y, min(value.y(), max_y))
        return QPointF(constrained_x, constrained_y)

    def update_attached_connections(self) -> None:
        for connection in self.connections:
            connection.update_position()

    def notify_parent_container_resized(self) -> None:
        if self.parent_container:
            self.parent_container.auto_resize_for_children()

    def initialize_breakpoint_marker(self) -> None:
        bounds = self.boundingRect()
        marker_size = 14.0
        marker_x = bounds.right() - marker_size / 2.0
        marker_y = bounds.center().y() - marker_size / 2.0

        self.breakpoint_marker = QGraphicsEllipseItem(
            marker_x,
            marker_y,
            marker_size,
            marker_size,
            self,
        )
        self.breakpoint_marker.setBrush(QBrush(PALETTE.runtime_log_error))
        self.breakpoint_marker.setPen(QPen(PALETTE.final_outcome_pen, 2))
        self.breakpoint_marker.setZValue(10.0)
        self.breakpoint_marker.setVisible(False)

    def set_breakpoint_marker(
        self,
        visible: bool,
        tooltip: str = "",
    ) -> None:
        marker = getattr(self, "breakpoint_marker", None)
        if marker is None:
            return
        marker.setVisible(bool(visible))
        marker.setToolTip(str(tooltip or ""))

    def initialize_start_indicator(self) -> None:
        bounds = self.boundingRect()

        marker_center_x = bounds.left() - 28.0
        marker_center_y = bounds.center().y()
        outer_size = 24.0
        inner_size = 16.0

        connector_start_x = marker_center_x + (outer_size / 2.0) - 1.0
        connector_end_x = bounds.right() - 8.0

        connector_pen = QPen(PALETTE.selection_pen, 3)
        connector_pen.setCapStyle(Qt.RoundCap)

        self.start_indicator_connector = QGraphicsLineItem(
            connector_start_x,
            marker_center_y,
            connector_end_x,
            marker_center_y,
            self,
        )
        self.start_indicator_connector.setPen(connector_pen)
        self.start_indicator_connector.setFlag(QGraphicsItem.ItemStacksBehindParent, True)
        self.start_indicator_connector.setAcceptedMouseButtons(Qt.NoButton)
        self.start_indicator_connector.setVisible(False)
        self.start_indicator_connector.setToolTip("Start state")

        outer_fill = QColor(PALETTE.runtime_highlight_fill)
        outer_fill.setAlpha(220)
        outer_pen = QColor(PALETTE.selection_pen)
        outer_pen.setAlpha(240)

        self.start_indicator_outer = QGraphicsEllipseItem(
            marker_center_x - (outer_size / 2.0),
            marker_center_y - (outer_size / 2.0),
            outer_size,
            outer_size,
            self,
        )
        self.start_indicator_outer.setBrush(QBrush(outer_fill))
        self.start_indicator_outer.setPen(QPen(outer_pen, 2))
        self.start_indicator_outer.setZValue(11.0)
        self.start_indicator_outer.setAcceptedMouseButtons(Qt.NoButton)
        self.start_indicator_outer.setVisible(False)
        self.start_indicator_outer.setToolTip("Start state")

        inner_fill = QColor(PALETTE.ui_window_bg)
        inner_fill.setAlpha(245)

        self.start_indicator_inner = QGraphicsEllipseItem(
            marker_center_x - (inner_size / 2.0),
            marker_center_y - (inner_size / 2.0),
            inner_size,
            inner_size,
            self,
        )
        self.start_indicator_inner.setBrush(QBrush(inner_fill))
        self.start_indicator_inner.setPen(QPen(PALETTE.runtime_highlight_pen, 1.5))
        self.start_indicator_inner.setZValue(11.0)
        self.start_indicator_inner.setAcceptedMouseButtons(Qt.NoButton)
        self.start_indicator_inner.setVisible(False)
        self.start_indicator_inner.setToolTip("Start state")

        arrow = QPolygonF(
            [
                QPointF(marker_center_x - 2.5, marker_center_y - 4.5),
                QPointF(marker_center_x - 2.5, marker_center_y + 4.5),
                QPointF(marker_center_x + 4.5, marker_center_y),
            ]
        )
        self.start_indicator_arrow = QGraphicsPolygonItem(arrow, self)
        self.start_indicator_arrow.setBrush(QBrush(PALETTE.runtime_highlight_pen))
        self.start_indicator_arrow.setPen(QPen(PALETTE.runtime_highlight_pen, 1.2))
        self.start_indicator_arrow.setZValue(11.0)
        self.start_indicator_arrow.setAcceptedMouseButtons(Qt.NoButton)
        self.start_indicator_arrow.setVisible(False)
        self.start_indicator_arrow.setToolTip("Start state")

        self.start_indicator_label = QGraphicsTextItem("START", self)
        start_font = self.start_indicator_label.font()
        start_font.setPointSize(8)
        start_font.setBold(True)
        self.start_indicator_label.setFont(start_font)
        self.start_indicator_label.setDefaultTextColor(PALETTE.selection_pen)
        label_rect = self.start_indicator_label.boundingRect()
        self.start_indicator_label.setPos(
            marker_center_x - (label_rect.width() / 2.0),
            marker_center_y - (outer_size / 2.0) - label_rect.height() - 3.0,
        )
        self.start_indicator_label.setZValue(11.0)
        self.start_indicator_label.setAcceptedMouseButtons(Qt.NoButton)
        self.start_indicator_label.setVisible(False)
        self.start_indicator_label.setToolTip("Start state")

        self.start_indicator_items = [
            self.start_indicator_connector,
            self.start_indicator_outer,
            self.start_indicator_inner,
            self.start_indicator_arrow,
            self.start_indicator_label,
        ]

        self.start_indicator_badge = self.start_indicator_connector

    def set_start_indicator(self, visible: bool, tooltip: str = "Start state") -> None:
        indicator_items = getattr(self, "start_indicator_items", None)
        if not indicator_items:
            return
        for item in indicator_items:
            item.setVisible(bool(visible))
            item.setToolTip(str(tooltip or ""))

    def update_selection_pen(
        self,
        selected: bool,
        default_pen: Optional[QPen] = None,
        default_pen_callback: Optional[Callable[[], Any]] = None,
    ) -> None:
        if selected:
            self.setPen(QPen(PALETTE.selection_pen, 4))
            return

        if default_pen is not None:
            self.setPen(default_pen)
            return

        if default_pen_callback is not None:
            default_pen_callback()
