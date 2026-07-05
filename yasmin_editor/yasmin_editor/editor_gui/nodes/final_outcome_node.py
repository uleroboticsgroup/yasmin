# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import Any, Optional

from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets
from yasmin_editor.editor_gui.colors import PALETTE
from yasmin_editor.editor_gui.nodes.base_node import BaseNodeMixin
from yasmin_editor.model.outcome import Outcome


class FinalOutcomeNode(BaseNodeMixin, QtWidgets.QGraphicsRectItem):
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

        self.setBrush(QtGui.QBrush(PALETTE.final_outcome_fill))
        self.setPen(QtGui.QPen(PALETTE.final_outcome_pen, 3))

        self.text: QtWidgets.QGraphicsTextItem = QtWidgets.QGraphicsTextItem(
            self.name, self
        )
        self.text.setDefaultTextColor(PALETTE.text_primary)
        font: QtGui.QFont = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.text.setFont(font)

        self.center_text_item(self.text, -self.text.boundingRect().height() / 2)

        self.update_tooltip()

    def update_tooltip(self) -> None:
        tooltip = self.description if self.description else self.name
        if self.instance_id:
            tooltip += f"\nView: {self.instance_id}"
        self.setToolTip(tooltip)

    def _on_name_changed(self, value: str) -> None:
        if hasattr(self, "text"):
            self.text.setPlainText(value)
            self.center_text_item(self.text, -self.text.boundingRect().height() / 2)
        self.update_tooltip()

    def _on_description_changed(self, value: str) -> None:
        self.update_tooltip()

    def _on_double_click(self, editor: Any, event: Any) -> None:
        editor.edit_final_outcome(self)

    def itemChange(
        self, change: QtWidgets.QGraphicsItem.GraphicsItemChange, value: Any
    ) -> Any:
        if (
            change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and isinstance(value, QtCore.QPointF)
        ):
            value = self.constrain_position_to_parent(value)
            self.update_attached_connections()

        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.notify_parent_container_resized()

        elif change == QtWidgets.QGraphicsItem.GraphicsItemChange.ItemSelectedChange:
            self.update_selection_pen(
                bool(value), QtGui.QPen(PALETTE.final_outcome_pen, 3)
            )

        return super().itemChange(change, value)

    def get_edge_point(self, target_pos: QtCore.QPointF) -> QtCore.QPointF:
        """Get the point on the node edge that is centered on the closest side."""
        return BaseNodeMixin.get_edge_point(self, target_pos)
