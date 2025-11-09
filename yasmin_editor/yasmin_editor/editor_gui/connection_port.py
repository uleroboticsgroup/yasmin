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

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsEllipseItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen, QBrush, QColor


class ConnectionPort(QGraphicsEllipseItem):
    """Connection port for drag-to-connect functionality."""

    def __init__(self, parent_state):
        super().__init__(-5, -5, 10, 10, parent_state)
        self.parent_state = parent_state
        self.setBrush(QBrush(QColor(100, 100, 255)))
        self.setPen(QPen(QColor(0, 0, 100), 1))

        from yasmin_editor.editor_gui.container_state_node import ContainerStateNode

        if isinstance(parent_state, ContainerStateNode):
            self.update_position_for_container()
        else:
            self.setPos(60, 0)

        self.setCursor(Qt.CrossCursor)
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)

    def update_position_for_container(self):
        """Update position for container parent."""
        from yasmin_editor.editor_gui.container_state_node import ContainerStateNode

        if isinstance(self.parent_state, ContainerStateNode):
            rect = self.parent_state.rect()
            center_y = rect.top() + rect.height() / 2
            self.setPos(rect.right(), center_y)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.scene() and self.scene().views():
                canvas = self.scene().views()[0]
                if hasattr(canvas, "start_connection_drag"):
                    canvas.start_connection_drag(self.parent_state, event)
                    event.accept()
                    return
        event.ignore()

    def mouseMoveEvent(self, event):
        event.accept()

    def mouseReleaseEvent(self, event):
        event.accept()
