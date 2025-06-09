# Copyright (C) 2023 Miguel Ángel González Santamarta
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

from typing import Set, Callable, Type, Any

from rclpy.node import Node
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT


class ServiceState(State):
    """
    A state representing a service call in a behavior tree.

    This state manages the communication with a ROS service, creating and sending
    requests, and handling responses. It can handle timeouts and custom outcomes.

    Attributes:
        _node (Node): The ROS node used to communicate with the service.
        _srv_name (str): The name of the service to call.
        _service_client (Client): The client used to call the service.
        _create_request_handler (Callable[[Blackboard], Any]): A function that creates the service request.
        _response_handler (Callable[[Blackboard, Any], str]): A function that processes the service response.
        _timeout (float): Timeout duration for the service call.
    """

    def __init__(
        self,
        srv_type: Type,
        srv_name: str,
        create_request_handler: Callable,
        outcomes: Set[str] = None,
        response_handler: Callable = None,
        callback_group: CallbackGroup = None,
        node: Node = None,
        timeout: float = None,
    ) -> None:
        """
        Initializes the ServiceState with the provided parameters.

        Parameters:
            srv_type (Type): The type of the service.
            srv_name (str): The name of the service to be called.
            create_request_handler (Callable[[Blackboard], Any]): A handler to create the request based on the blackboard data.
            outcomes (Set[str], optional): A set of additional outcomes for this state.
            response_handler (Callable[[Blackboard, Any], str], optional): A handler to process the service response.
            callback_group (CallbackGroup, optional): The callback group for the client.
            node (Node, optional): A ROS node instance; if None, a default instance is used.
            timeout (float, optional): Timeout duration for waiting on the service.

        Raises:
            ValueError: If the create_request_handler is not provided.
        """

        ## A function that creates the service request.
        self._create_request_handler: Callable[[Blackboard], Any] = create_request_handler
        ## A function that processes the service response.
        self._response_handler: Callable[[Blackboard, Any], str] = response_handler

        ## Timeout duration for the service call.
        self._timeout: float = timeout

        _outcomes = [SUCCEED, ABORT]

        if self._timeout:
            _outcomes.append(TIMEOUT)

        if outcomes:
            _outcomes = _outcomes + outcomes

        ## The ROS node used to communicate with the service.
        self._node = node

        if self._node is None:
            self._node: Node = YasminNode.get_instance()

        ## The name of the service to call.
        self._srv_name: str = srv_name

        ## The client used to call the service.
        self._service_client: Client = self._node.create_client(
            srv_type, srv_name, callback_group=callback_group
        )

        if not self._create_request_handler:
            raise ValueError("create_request_handler is needed")

        super().__init__(_outcomes)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the service call.

        This method prepares the request using the provided blackboard,
        waits for the service to become available, and sends the request.
        It also handles the response and can process outcomes based on the
        response handler.

        Parameters:
            blackboard (Blackboard): The blackboard object that holds the data for request creation.

        Returns:
            str: The outcome of the state execution, which can be SUCCEED, ABORT, or TIMEOUT.

        Exceptions:
            Exception: Catches all exceptions during the service call and returns ABORT.
        """
        request = self._create_request_handler(blackboard)

        yasmin.YASMIN_LOG_INFO(f"Waiting for service '{self._srv_name}'")
        srv_available = self._service_client.wait_for_service(timeout_sec=self._timeout)

        if not srv_available:
            yasmin.YASMIN_LOG_WARN(
                f"Timeout reached, service '{self._srv_name}' is not available"
            )
            return TIMEOUT

        try:
            yasmin.YASMIN_LOG_INFO(f"Sending request to service '{self._srv_name}'")
            response = self._service_client.call(request)

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Service call failed: {e}")
            return ABORT

        if self._response_handler:
            outcome = self._response_handler(blackboard, response)
            return outcome

        return SUCCEED
