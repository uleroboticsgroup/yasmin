# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from typing import Callable, Optional

from rclpy.node import Node

import yasmin
from yasmin_ros.basic_outcomes import CANCEL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode


def resolve_node(node: Optional[Node] = None) -> Node:
    if node is not None:
        return node
    return YasminNode.get_instance()


def wait_with_retry(
    condition_fn: Callable[[], bool],
    max_retry: int,
    log_msg: str,
    cancel_check: Optional[Callable[[], bool]] = None,
    timeout: Optional[float] = None,
) -> Optional[str]:
    retry_count = 0
    while not condition_fn():
        if cancel_check is not None and cancel_check():
            return CANCEL
        if retry_count >= max_retry:
            return TIMEOUT
        retry_count += 1
        yasmin.YASMIN_LOG_WARN(f"{log_msg} ({retry_count}/{max_retry})")
    return None
