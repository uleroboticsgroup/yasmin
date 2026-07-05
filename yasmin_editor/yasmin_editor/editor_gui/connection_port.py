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

from typing import TYPE_CHECKING, Union

from yasmin_editor.qt_compat import Qt, QtCore, QtGui, QtWidgets

from yasmin_editor.editor_gui.colors import PALETTE

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.nodes.container_state_node import ContainerStateNode
    from yasmin_editor.editor_gui.nodes.final_outcome_node import FinalOutcomeNode
    from yasmin_editor.editor_gui.nodes.state_node import StateNode


class ConnectionPort(QtWidgets.QGraphicsEllipseItem):
    """Connection port for drag-to-connect functionality."""

    def __init__(
        self, parent_state: Union["StateNode", "ContainerStateNode", "FinalOutcomeNode"]
    ) -> None:
        super().__init__(-5, -5, 10, 10, parent_state)
        self.parent_state: Union[
            "StateNode", "ContainerStateNode", "FinalOutcomeNode"
        ] = parent_state
        self.setBrush(QtGui.QBrush(PALETTE.connection_port_fill))
        self.setPen(QtGui.QPen(PALETTE.connection_port_pen, 1))

        from yasmin_editor.editor_gui.nodes.container_state_node import (
            ContainerStateNode,
        )

        if isinstance(parent_state, ContainerStateNode):
            self.update_position_for_container()
        else:
            self.setPos(60, 0)

        self.setCursor(Qt.CursorShape.CrossCursor)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)

    def update_position_for_container(self) -> None:
        """Update position for container parent."""
        from yasmin_editor.editor_gui.nodes.container_state_node import (
            ContainerStateNode,
        )

        if isinstance(self.parent_state, ContainerStateNode):
            rect = self.parent_state.rect()
            center_y: float = rect.top() + rect.height() / 2
            self.setPos(rect.right(), center_y)

    def mousePressEvent(self, event: QtCore.QEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            if self.scene() and self.scene().views():
                canvas = self.scene().views()[0]
                if hasattr(canvas, "start_connection_drag"):
                    canvas.start_connection_drag(self.parent_state, event)
                    event.accept()
                    return
        event.ignore()

    def mouseMoveEvent(self, event: QtCore.QEvent) -> None:
        event.accept()

    def mouseReleaseEvent(self, event: QtCore.QEvent) -> None:
        event.accept()
