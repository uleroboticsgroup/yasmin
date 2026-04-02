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

"""Geometry helpers for connection line routing and arrow creation."""

import math
from typing import Tuple

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPolygonF

ARROW_SIZE: float = 12.0
DEFAULT_DIRECTION = QPointF(1.0, 0.0)
ZERO_POINT = QPointF(0.0, 0.0)
VECTOR_EPSILON = 1e-9


def vector_length(vector: QPointF) -> float:
    """Return the Euclidean length of a 2D vector."""
    return math.hypot(vector.x(), vector.y())


def normalize_vector(vector: QPointF) -> QPointF:
    """Return a unit vector for the given vector.

    A zero vector is mapped to ``(0, 0)`` so callers can explicitly handle the
    degenerate case.
    """
    length = vector_length(vector)
    if length <= VECTOR_EPSILON:
        return QPointF(0.0, 0.0)
    return QPointF(vector.x() / length, vector.y() / length)


def offset_point(point: QPointF, direction: QPointF, distance: float) -> QPointF:
    """Return a point translated along the given direction."""
    return QPointF(
        point.x() + direction.x() * distance,
        point.y() + direction.y() * distance,
    )


def compute_arrow_direction(
    tip_pos: QPointF,
    control_pos: QPointF,
    fallback_direction: QPointF,
) -> QPointF:
    """Compute the direction that should be used for the arrow head.

    The direction is primarily derived from the last path segment. If that is
    degenerate, the fallback direction is used instead.
    """
    direction = normalize_vector(
        QPointF(tip_pos.x() - control_pos.x(), tip_pos.y() - control_pos.y())
    )
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = normalize_vector(fallback_direction)
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = DEFAULT_DIRECTION
    return direction


def build_arrow_polygon(
    target_pos: QPointF,
    target_direction: QPointF,
) -> Tuple[QPolygonF, QPointF, float]:
    """Build the arrow polygon and return the shaft end point and angle.

    Returns:
        A tuple containing the arrow polygon, the line end point just before the
        arrow head, and the arrow angle in radians.
    """
    direction = normalize_vector(target_direction)
    if vector_length(direction) <= VECTOR_EPSILON:
        direction = DEFAULT_DIRECTION

    angle = math.atan2(direction.y(), direction.x())
    arrow_p1 = target_pos - QPointF(
        math.cos(angle - math.pi / 6.0) * ARROW_SIZE,
        math.sin(angle - math.pi / 6.0) * ARROW_SIZE,
    )
    arrow_p2 = target_pos - QPointF(
        math.cos(angle + math.pi / 6.0) * ARROW_SIZE,
        math.sin(angle + math.pi / 6.0) * ARROW_SIZE,
    )
    polygon = QPolygonF([target_pos, arrow_p1, arrow_p2])

    base_offset = ARROW_SIZE * math.cos(math.pi / 6.0)
    line_end = QPointF(
        target_pos.x() - direction.x() * base_offset,
        target_pos.y() - direction.y() * base_offset,
    )
    return polygon, line_end, angle


def use_upward_label_stack(from_center: QPointF, to_center: QPointF) -> bool:
    """Return ``True`` if stacked labels should be placed above the path."""
    if abs(from_center.x() - to_center.x()) > 1e-6:
        return from_center.x() < to_center.x()
    return from_center.y() < to_center.y()
