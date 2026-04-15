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

from typing import Any, Optional

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QBrush, QFont, QPen
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsRectItem, QGraphicsTextItem

from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.outcome import Outcome


class FinalOutcomeNode(QGraphicsRectItem, BaseNodeMixin):
    """Graphical representation of a final outcome."""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        inside_container: bool = False,
        description: str = "",
        model: Optional[Outcome] = None,
        instance_id: str = "",
    ) -> None:
        super().__init__(-60, -25, 120, 50)
        self.model: Outcome = model or Outcome(name=name, description=description)
        self.instance_id = instance_id

        self._initialize_base_node_graphics(x, y)

        self.setBrush(QBrush(PALETTE.final_outcome_fill))
        self.setPen(QPen(PALETTE.final_outcome_pen, 3))

        self.text: QGraphicsTextItem = QGraphicsTextItem(self.name, self)
        self.text.setDefaultTextColor(PALETTE.text_primary)
        font: QFont = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

        self.update_tooltip()

    @property
    def name(self) -> str:
        return self.model.name

    @name.setter
    def name(self, value: str) -> None:
        self.model.name = value
        if hasattr(self, "text"):
            self.text.setPlainText(value)
            self.center_text_item(self.text, -self.text.boundingRect().height() / 2)
        self.update_tooltip()

    @property
    def description(self) -> str:
        return self.model.description

    @description.setter
    def description(self, value: str) -> None:
        self.model.description = value
        self.update_tooltip()

    def update_tooltip(self) -> None:
        tooltip = self.description if self.description else self.name
        if self.instance_id:
            tooltip += f"\nView: {self.instance_id}"
        self.setToolTip(tooltip)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        """Handle double-click to edit final outcome metadata."""
        if self.scene() and self.scene().views():
            canvas = self.scene().views()[0]
            if hasattr(canvas, "editor_ref") and canvas.editor_ref:
                self.setSelected(True)
                canvas.editor_ref.edit_final_outcome(self)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if change == QGraphicsItem.ItemPositionChange and isinstance(value, QPointF):
            value = self.constrain_position_to_parent(value)
            self.update_attached_connections()

        elif change == QGraphicsItem.ItemPositionHasChanged:
            self.notify_parent_container_resized()

        elif change == QGraphicsItem.ItemSelectedChange:
            self.update_selection_pen(bool(value), QPen(PALETTE.final_outcome_pen, 3))

        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QPointF) -> QPointF:
        """Get the point on the node edge that is centered on the closest side."""
        return BaseNodeMixin.get_edge_point(self, target_pos)
