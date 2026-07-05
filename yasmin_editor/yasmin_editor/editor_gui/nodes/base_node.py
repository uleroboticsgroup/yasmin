# Copyright (C) 2026 Maik Knof
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

import math
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Set

from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets
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
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(
            QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True
        )

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

    def center_text_item(self, item: QtWidgets.QGraphicsTextItem, y: float) -> None:
        text_rect = item.boundingRect()
        item.setPos(-text_rect.width() / 2.0, y)

    def get_connection_point(self) -> QtCore.QPointF:
        return self.sceneBoundingRect().center()

    def get_edge_point(self, target_pos: QtCore.QPointF) -> QtCore.QPointF:
        bounds = self.sceneBoundingRect()
        center = bounds.center()
        dx = target_pos.x() - center.x()
        dy = target_pos.y() - center.y()

        if math.isclose(dx, 0.0, abs_tol=1e-9) and math.isclose(dy, 0.0, abs_tol=1e-9):
            return center

        half_width = max(bounds.width() / 2.0, 1.0)
        half_height = max(bounds.height() / 2.0, 1.0)
        scale = 1.0 / max(abs(dx) / half_width, abs(dy) / half_height, 1e-9)
        return QtCore.QPointF(center.x() + dx * scale, center.y() + dy * scale)

    def constrain_position_to_parent(
        self,
        value: QtCore.QPointF,
        top_margin: float = 40.0,
        side_margin: float = 10.0,
        bottom_margin: float = 10.0,
    ) -> QtCore.QPointF:
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
        return QtCore.QPointF(constrained_x, constrained_y)

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

        self.breakpoint_marker = QtWidgets.QGraphicsEllipseItem(
            marker_x,
            marker_y,
            marker_size,
            marker_size,
            self,
        )
        self.breakpoint_marker.setBrush(QtGui.QBrush(PALETTE.runtime_log_error))
        self.breakpoint_marker.setPen(QtGui.QPen(PALETTE.final_outcome_pen, 2))
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

        connector_pen = QtGui.QPen(PALETTE.start_indicator_connector, 3)
        connector_pen.setCapStyle(Qt.PenCapStyle.RoundCap)

        self.start_indicator_connector = QtWidgets.QGraphicsLineItem(
            connector_start_x,
            marker_center_y,
            connector_end_x,
            marker_center_y,
            self,
        )
        self.start_indicator_connector.setPen(connector_pen)
        self.start_indicator_connector.setFlag(
            QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemStacksBehindParent, True
        )
        self.start_indicator_connector.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.start_indicator_connector.setVisible(False)
        self.start_indicator_connector.setToolTip("Start state")

        outer_fill = QtGui.QColor(PALETTE.start_indicator_outer_fill)
        outer_fill.setAlpha(220)
        outer_pen = QtGui.QColor(PALETTE.start_indicator_outer_pen)
        outer_pen.setAlpha(240)

        self.start_indicator_outer = QtWidgets.QGraphicsEllipseItem(
            marker_center_x - (outer_size / 2.0),
            marker_center_y - (outer_size / 2.0),
            outer_size,
            outer_size,
            self,
        )
        self.start_indicator_outer.setBrush(QtGui.QBrush(outer_fill))
        self.start_indicator_outer.setPen(QtGui.QPen(outer_pen, 2))
        self.start_indicator_outer.setZValue(11.0)
        self.start_indicator_outer.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.start_indicator_outer.setVisible(False)
        self.start_indicator_outer.setToolTip("Start state")

        inner_fill = QtGui.QColor(PALETTE.start_indicator_inner_fill)
        inner_fill.setAlpha(245)

        self.start_indicator_inner = QtWidgets.QGraphicsEllipseItem(
            marker_center_x - (inner_size / 2.0),
            marker_center_y - (inner_size / 2.0),
            inner_size,
            inner_size,
            self,
        )
        self.start_indicator_inner.setBrush(QtGui.QBrush(inner_fill))
        self.start_indicator_inner.setPen(
            QtGui.QPen(PALETTE.start_indicator_inner_pen, 1.5)
        )
        self.start_indicator_inner.setZValue(11.0)
        self.start_indicator_inner.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.start_indicator_inner.setVisible(False)
        self.start_indicator_inner.setToolTip("Start state")

        arrow = QtGui.QPolygonF(
            [
                QtCore.QPointF(marker_center_x - 2.5, marker_center_y - 4.5),
                QtCore.QPointF(marker_center_x - 2.5, marker_center_y + 4.5),
                QtCore.QPointF(marker_center_x + 4.5, marker_center_y),
            ]
        )
        self.start_indicator_arrow = QtWidgets.QGraphicsPolygonItem(arrow, self)
        self.start_indicator_arrow.setBrush(QtGui.QBrush(PALETTE.start_indicator_arrow))
        self.start_indicator_arrow.setPen(QtGui.QPen(PALETTE.start_indicator_arrow, 1.2))
        self.start_indicator_arrow.setZValue(11.0)
        self.start_indicator_arrow.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.start_indicator_arrow.setVisible(False)
        self.start_indicator_arrow.setToolTip("Start state")

        self.start_indicator_label = QtWidgets.QGraphicsTextItem("START", self)
        start_font = self.start_indicator_label.font()
        start_font.setPointSize(8)
        start_font.setBold(True)
        self.start_indicator_label.setFont(start_font)
        self.start_indicator_label.setDefaultTextColor(PALETTE.start_indicator_label)
        label_rect = self.start_indicator_label.boundingRect()
        self.start_indicator_label.setPos(
            marker_center_x - (label_rect.width() / 2.0),
            marker_center_y - (outer_size / 2.0) - label_rect.height() - 3.0,
        )
        self.start_indicator_label.setZValue(11.0)
        self.start_indicator_label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
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
        default_pen: Optional[QtGui.QPen] = None,
        default_pen_callback: Optional[Callable[[], Any]] = None,
    ) -> None:
        if selected:
            self.setPen(QtGui.QPen(PALETTE.selection_pen, 4))
            return

        if default_pen is not None:
            self.setPen(default_pen)
            return

        if default_pen_callback is not None:
            default_pen_callback()

    @property
    def name(self) -> str:
        return self.model.name

    @name.setter
    def name(self, value: str) -> None:
        self.model.name = value
        self._on_name_changed(value)

    @property
    def description(self) -> str:
        return self.model.description

    @description.setter
    def description(self, value: str) -> None:
        self.model.description = value
        self._on_description_changed(value)

    @property
    def remappings(self) -> Dict[str, str]:
        return self.model.remappings

    @remappings.setter
    def remappings(self, value: Dict[str, str]) -> None:
        self.model.remappings.clear()
        self.model.remappings.update(value or {})

    @property
    def parameter_mappings(self) -> Dict[str, str]:
        return self.model.parameter_mappings

    @parameter_mappings.setter
    def parameter_mappings(self, value: Dict[str, str]) -> None:
        self.model.parameter_mappings.clear()
        self.model.parameter_mappings.update(value or {})

    def _on_name_changed(self, value: str) -> None:
        pass

    def _on_description_changed(self, value: str) -> None:
        pass

    def mouseDoubleClickEvent(self, event: Any) -> None:
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                self._on_double_click(canvas.editor_ref, event)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def _on_double_click(self, editor: Any, event: Any) -> None:
        pass

    def contextMenuEvent(self, event: Any) -> None:
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                if self._on_context_menu(canvas.editor_ref, event):
                    event.accept()
                    return
        super().contextMenuEvent(event)

    def _on_context_menu(self, editor: Any, event: Any) -> bool:
        return False
