# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import time
from typing import List, Callable, Union, Type

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile

from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import TIMEOUT


class MonitorState(State):

    _node: Node
    _sub: Subscription
    _monitor_handler: Callable
    _topic_name: str

    msg_list: List = []
    msg_queue: int
    time_to_wait: float = 0.001
    monitoring: bool = False
    _timeout: int

    def __init__(
        self,
        msg_type: Type,
        topic_name: str,
        outcomes: List[str],
        monitor_handler: Callable,
        qos: Union[QoSProfile, int] = 10,
        msg_queue: int = 10,
        node: Node = None,
        timeout: int = None,
    ) -> None:

        self._topic_name = topic_name

        self._timeout = timeout
        if not timeout is None:
            outcomes = [TIMEOUT] + outcomes
        super().__init__(outcomes)

        self._monitor_handler = monitor_handler
        self.msg_queue = msg_queue

        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        self._sub = self._node.create_subscription(
            msg_type, topic_name, self.__callback, qos
        )

    def __callback(self, msg) -> None:

        if self.monitoring:
            self.msg_list.append(msg)

            if len(self.msg_list) >= self.msg_queue:
                self.msg_list.pop(0)

    def execute(self, blackboard: Blackboard) -> str:

        elapsed_time = 0
        self.msg_list = []
        self.monitoring = True

        while not self.msg_list:
            time.sleep(self.time_to_wait)

            if not self._timeout is None:

                if elapsed_time >= self._timeout:
                    self.monitoring = False
                    self._node.get_logger().warn(
                        f"Timeout reached, topic '{self._topic_name}' is not available"
                    )
                    return TIMEOUT

                elapsed_time += self.time_to_wait

        self._node.get_logger().info(f"Processing msg from topic '{self._topic_name}'")
        outcome = self._monitor_handler(blackboard, self.msg_list.pop(0))

        self.monitoring = False
        return outcome
