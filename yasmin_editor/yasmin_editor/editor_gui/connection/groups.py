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
