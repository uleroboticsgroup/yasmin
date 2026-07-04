# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from threading import Event
from typing import Callable, Optional, Set

from rclpy.node import Node

import yasmin
from yasmin_ros.basic_outcomes import CANCEL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode


def resolve_node(node: Optional[Node] = None) -> Node:
    if node is not None:
        return node
    return YasminNode.get_instance()


def setup_outcomes(
    outcomes: Set[str],
    base_outcomes: Set[str],
    add_timeout: bool = False,
) -> Set[str]:
    outcomes = set(outcomes)
    outcomes.update(base_outcomes)
    if add_timeout:
        outcomes.add(TIMEOUT)
    return outcomes


def cancel_with_event(event: Event) -> None:
    event.set()


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
