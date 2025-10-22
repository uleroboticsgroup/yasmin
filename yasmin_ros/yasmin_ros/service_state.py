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

from threading import Event
from typing import Set, Callable, Type, Any

from rclpy.node import Node
from rclpy.task import Future
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros.ros_clients_cache import ROSClientsCache


class ServiceState(State):
    """
    A state class that interacts with a ROS 2 service.

    This class manages communication with a specified ROS 2 service,
    allowing it to send requests and handle responses. It extends
    the base State class.

    Attributes:
        _node (Node): The ROS 2 node used to communicate with the service.
        _srv_name (str): The name of the service to call.
        _service_client (Client): The client used to call the service.
        _create_request_handler (Callable[[Blackboard], Any]): Function to create service requests.
        _response (Any): The response received from the service.
        _response_handler (Callable[[Blackboard, Any], str]): Function to handle service responses.
        _wait_timeout (float): Maximum wait time for service availability.
        _response_timeout (float): Timeout for the service response.
        _maximum_retry (int): Maximum number of retries.
        _response_received_event (Event): Event to signal when the service response is received.
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
        wait_timeout: float = None,
        response_timeout: float = None,
        maximum_retry: int = 3,
    ) -> None:
        """
        Initializes the ServiceState with the provided parameters.

        Args:
            srv_type (Type): The type of the service.
            srv_name (str): The name of the service to call.
            create_request_handler (Callable[[Blackboard], Any]): Function to create a service request.
            outcomes (Set[str], optional): A set of possible outcomes for this state.
            response_handler (Callable[[Blackboard, Any], str], optional): Function to handle the service response.
            callback_group (CallbackGroup, optional): The callback group for the service client.
            node (Node, optional): A ROS 2 node instance; if None, a default instance is used.
            wait_timeout (float, optional): Maximum time to wait for the service to become available. Default is None (wait indefinitely).
            response_timeout (float, optional): Maximum time to wait for the service response. Default is None (wait indefinitely).
            maximum_retry (int, optional): Maximum retries of the service if it returns timeout. Default is 3.

        Raises:
            ValueError: If the create_request_handler is not provided.
        """

        ## Function to create service requests.
        self._create_request_handler: Callable[[Blackboard], Any] = create_request_handler
        ## Function to handle service responses.
        self._response_handler: Callable[[Blackboard, Any], str] = response_handler

        ## Maximum wait time for service availability.
        self._wait_timeout: float = wait_timeout
        ## Timeout for the service response.
        self._response_timeout: float = response_timeout

        _outcomes = [SUCCEED, ABORT]

        if self._wait_timeout or self._response_timeout:
            _outcomes.append(TIMEOUT)

        if outcomes:
            _outcomes = _outcomes + outcomes

        ## The ROS 2 node used to communicate with the service.
        self._node = node

        if self._node is None:
            self._node: Node = YasminNode.get_instance()

        ## Name of the service.
        self._srv_name: str = srv_name

        ## Shared pointer to the service client.
        self._service_client: Client = ROSClientsCache.get_or_create_service_client(
            self._node,
            srv_type,
            srv_name,
            callback_group=callback_group,
        )

        ## The response received from the service.
        self._response: Any = None

        if not self._create_request_handler:
            raise ValueError("create_request_handler is needed")

        ## Maximum number of retries.
        self._maximum_retry: int = maximum_retry

        ## Event to signal when the service response is received.
        self._response_received_event: Event = Event()

        super().__init__(_outcomes)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the service call and handle the response.

        This method creates a request based on the blackboard data, waits for the
        service to become available, sends the request asynchronously, and waits for
        the response using a threading Event.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard containing data for
                request creation.

        Returns:
            str: The outcome of the service call, which can be SUCCEED, ABORT, or TIMEOUT.
        """
        request = self._create_request_handler(blackboard)
        retry_count = 0

        yasmin.YASMIN_LOG_INFO(f"Waiting for service '{self._srv_name}'")

        while not self._service_client.wait_for_service(timeout_sec=self._wait_timeout):
            yasmin.YASMIN_LOG_WARN(
                f"Timeout reached, service '{self._srv_name}' is not available"
            )
            if retry_count < self._maximum_retry:
                retry_count += 1
                yasmin.YASMIN_LOG_WARN(
                    f"Retrying to connect to service '{self._srv_name}' ({retry_count}/{self._maximum_retry})"
                )
            else:
                return TIMEOUT

        try:
            yasmin.YASMIN_LOG_INFO(f"Sending request to service '{self._srv_name}'")

            # Clear the event for this service call
            self._response_received_event.clear()

            future = self._service_client.call_async(request)
            future.add_done_callback(self.response_callback)

            # Wait for the future to complete with optional timeout
            while not self._response_received_event.wait(self._response_timeout):
                yasmin.YASMIN_LOG_WARN(
                    f"Timeout reached while waiting for response from service '{self._srv_name}'"
                )

                if retry_count < self._maximum_retry:
                    retry_count += 1
                    yasmin.YASMIN_LOG_WARN(
                        f"Retrying to wait for service '{self._srv_name}' response ({retry_count}/{self._maximum_retry})"
                    )
                else:
                    return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Service call failed: {e}")
            return ABORT

        if self._response_handler:
            outcome = self._response_handler(blackboard, self._response)
            return outcome

        return SUCCEED

    def response_callback(self, future: Future) -> None:
        """
        Callback function to process the service response.

        This method is called when the service response is received. It stores
        the response and signals the waiting thread by setting the event.

        Args:
            future (Future): The future object containing the service response.
        """
        self._response = future.result()
        self._response_received_event.set()
