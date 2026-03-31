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

"""Helpers for laying out grouped connection labels."""

from typing import TYPE_CHECKING, List

from PyQt5.QtCore import QPointF, QRectF

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine


LABEL_PADDING: float = 4.0
LABEL_SPACING: float = 2.0


def layout_stacked_labels(
    group: List["ConnectionLine"],
    anchor_point: QPointF,
    stack_direction: str = "center",
) -> None:
    """Lay out the labels of a grouped connection as a vertical stack."""
    if not group:
        return

    box_heights: List[float] = []
    total_height: float = 0.0

    for connection in group:
        label_rect = connection.label.boundingRect()
        box_height = label_rect.height() + LABEL_PADDING * 2
        box_heights.append(box_height)
        total_height += box_height

    total_height += LABEL_SPACING * max(0, len(group) - 1)
    if stack_direction == "down":
        current_top = anchor_point.y() - total_height
    elif stack_direction == "up":
        current_top = anchor_point.y()
    else:
        current_top = anchor_point.y() - total_height / 2.0

    for connection, box_height in zip(group, box_heights):
        label_rect = connection.label.boundingRect()
        box_width = label_rect.width() + LABEL_PADDING * 2
        box_left = anchor_point.x() - box_width / 2.0
        connection.label_bg.setRect(QRectF(box_left, current_top, box_width, box_height))
        connection.label.setPos(
            box_left + LABEL_PADDING,
            current_top + LABEL_PADDING,
        )
        current_top += box_height + LABEL_SPACING
