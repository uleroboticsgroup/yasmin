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


from typing import List, Callable, Type, Any

from rclpy.node import Node
from rclpy.client import Client

from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT


class ServiceState(State):

    _node: Node
    _srv_name: str
    _service_client: Client
    _create_request_handler: Callable
    _response_handler: Callable
    _timeout: float

    def __init__(
        self,
        srv_type: Type,
        srv_name: str,
        create_request_handler: Callable,
        outcomes: List[str] = None,
        response_handler: Callable = None,
        node: Node = None,
        timeout: float = None,
    ) -> None:

        self._srv_name = srv_name
        _outcomes = [SUCCEED, ABORT]

        self._timeout = timeout
        if self._timeout:
            _outcomes.append(TIMEOUT)

        if outcomes:
            _outcomes = _outcomes + outcomes

        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        self._service_client = self._node.create_client(srv_type, srv_name)

        self._create_request_handler = create_request_handler
        self._response_handler = response_handler

        if not self._create_request_handler:
            raise Exception("create_request_handler is needed")

        super().__init__(_outcomes)

    def execute(self, blackboard: Blackboard) -> str:

        request = self._create_request_handler(blackboard)

        self._node.get_logger().info(f"Waiting for service '{self._srv_name}'")
        srv_available = self._service_client.wait_for_service(timeout_sec=self._timeout)

        if not srv_available:
            self._node.get_logger().warn(
                f"Timeout reached, service '{self._srv_name}' is not available"
            )
            return TIMEOUT

        try:
            self._node.get_logger().info(
                f"Sending request to service '{self._srv_name}'"
            )
            response = self._service_client.call(request)
        except:
            return ABORT

        if self._response_handler:
            outcome = self._response_handler(blackboard, response)
            return outcome

        return SUCCEED
