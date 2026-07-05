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

from typing import TYPE_CHECKING, List

from yasmin_editor.qt_compat import QtCore

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine


LABEL_PADDING: float = 4.0
LABEL_SPACING: float = 2.0


def layout_stacked_labels(
    group: List["ConnectionLine"],
    anchor_point: QtCore.QPointF,
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
        connection.label_bg.setRect(
            QtCore.QRectF(box_left, current_top, box_width, box_height)
        )
        connection.label.setPos(
            box_left + LABEL_PADDING,
            current_top + LABEL_PADDING,
        )
        current_top += box_height + LABEL_SPACING
