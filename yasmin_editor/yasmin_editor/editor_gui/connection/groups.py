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

"""Grouping helpers for connection lines that share endpoints."""

from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.connection_line import ConnectionLine


def get_direction_group(connection: "ConnectionLine") -> List["ConnectionLine"]:
    """Return all connections with the same source and target nodes."""
    group: List["ConnectionLine"] = []
    for candidate in list(connection.from_node.connections) + list(
        connection.to_node.connections
    ):
        if (
            candidate.from_node == connection.from_node
            and candidate.to_node == connection.to_node
            and candidate not in group
        ):
            group.append(candidate)
    group.sort(key=lambda item: item.outcome)
    return group


def get_opposite_direction_group(
    connection: "ConnectionLine",
) -> List["ConnectionLine"]:
    """Return all connections running in the opposite direction."""
    group: List["ConnectionLine"] = []
    for candidate in list(connection.from_node.connections) + list(
        connection.to_node.connections
    ):
        if (
            candidate.from_node == connection.to_node
            and candidate.to_node == connection.from_node
            and candidate not in group
        ):
            group.append(candidate)
    group.sort(key=lambda item: item.outcome)
    return group


def get_self_loop_group(connection: "ConnectionLine") -> List["ConnectionLine"]:
    """Return all self-loop connections on the source node."""
    group: List["ConnectionLine"] = []
    for candidate in connection.from_node.connections:
        if (
            candidate.from_node == candidate.to_node
            and candidate.from_node == connection.from_node
            and candidate not in group
        ):
            group.append(candidate)
    group.sort(key=lambda item: item.outcome)
    return group
