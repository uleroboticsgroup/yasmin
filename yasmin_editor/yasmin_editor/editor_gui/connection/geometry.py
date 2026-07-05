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
from typing import Tuple

from yasmin_editor.qt_compat import QtCore, QtGui

ARROW_SIZE: float = 12.0
DEFAULT_DIRECTION = QtCore.QPointF(1.0, 0.0)
ZERO_POINT = QtCore.QPointF(0.0, 0.0)
VECTOR_EPSILON = 1e-9


def vector_length(vector: QtCore.QPointF) -> float:
    """Return the Euclidean length of a 2D vector."""
    return math.hypot(vector.x(), vector.y())


def normalize_vector(vector: QtCore.QPointF) -> QtCore.QPointF:
    """Return a unit vector for the given vector.

    A zero vector is mapped to ``(0, 0)`` so callers can explicitly handle the
    degenerate case.
    """
    length = vector_length(vector)
    if length <= VECTOR_EPSILON:
        return QtCore.QPointF(0.0, 0.0)
    return QtCore.QPointF(vector.x() / length, vector.y() / length)


def offset_point(
    point: QtCore.QPointF, direction: QtCore.QPointF, distance: float
) -> QtCore.QPointF:
    """Return a point translated along the given direction."""
    return QtCore.QPointF(
        point.x() + direction.x() * distance,
        point.y() + direction.y() * distance,
    )


def compute_arrow_direction(
    tip_pos: QtCore.QPointF,
    control_pos: QtCore.QPointF,
    fallback_direction: QtCore.QPointF,
) -> QtCore.QPointF:
    """Compute the direction that should be used for the arrow head.

    The direction is primarily derived from the last path segment. If that is
    degenerate, the fallback direction is used instead.
    """
    direction = normalize_vector(
        QtCore.QPointF(tip_pos.x() - control_pos.x(), tip_pos.y() - control_pos.y())
    )
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = normalize_vector(fallback_direction)
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = DEFAULT_DIRECTION
    return direction


def build_arrow_polygon(
    target_pos: QtCore.QPointF,
    target_direction: QtCore.QPointF,
) -> Tuple[QtGui.QPolygonF, QtCore.QPointF, float]:
    """Build the arrow polygon and return the shaft end point and angle.

    Returns:
        A tuple containing the arrow polygon, the line end point just before the
        arrow head, and the arrow angle in radians.
    """
    direction = normalize_vector(target_direction)
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = DEFAULT_DIRECTION

    angle = math.atan2(direction.y(), direction.x())
    arrow_p1 = target_pos - QtCore.QPointF(
        math.cos(angle - math.pi / 6.0) * ARROW_SIZE,
        math.sin(angle - math.pi / 6.0) * ARROW_SIZE,
    )
    arrow_p2 = target_pos - QtCore.QPointF(
        math.cos(angle + math.pi / 6.0) * ARROW_SIZE,
        math.sin(angle + math.pi / 6.0) * ARROW_SIZE,
    )
    polygon = QtGui.QPolygonF([target_pos, arrow_p1, arrow_p2])

    base_offset = ARROW_SIZE * math.cos(math.pi / 6.0)
    line_end = QtCore.QPointF(
        target_pos.x() - direction.x() * base_offset,
        target_pos.y() - direction.y() * base_offset,
    )
    return polygon, line_end, angle


def use_upward_label_stack(
    from_center: QtCore.QPointF, to_center: QtCore.QPointF
) -> bool:
    """Return ``True`` if stacked labels should be placed above the path."""
    if abs(from_center.x() - to_center.x()) > 1e-6:
        return from_center.x() < to_center.x()
    return from_center.y() < to_center.y()
