# Copyright (C) 2023 Miguel Ángel González Santamarta
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

from threading import RLock, Event
from typing import Set, Callable, Type, Any

from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup
from action_msgs.msg import GoalStatus

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros.ros_state_utils import resolve_node, wait_with_retry, setup_outcomes


class ActionState(State):
    """
    A state class for handling ROS 2 action client operations.

    This class encapsulates the behavior of a ROS 2 action client within a YASMIN
    state. It allows the creation and management of goals, feedback, and results
    associated with an action server.
    """

    def __init__(
        self,
        action_type: Type,
        action_name: str,
        create_goal_handler: Callable,
        outcomes: Set[str] = set(),
        result_handler: Callable = None,
        feedback_handler: Callable = None,
        callback_group: CallbackGroup = None,
        node: Node = None,
        wait_timeout: float = None,
        response_timeout: float = None,
        maximum_retry: int = 3,
    ) -> None:
        """
        Construct an ActionState with a specific action name and goal handler.

        This constructor initializes the action state with a specified action name,
        goal handler, and optional timeout.

        Args:
            action_type (Type): The type of the action to be executed.
            action_name (str): The name of the action to communicate with.
            create_goal_handler (Callable[[Blackboard], Any]): A function that creates a goal for the action.
            outcomes (Set[str], optional): A set of possible outcomes for this action state.
            result_handler (Callable[[Blackboard, Any], str], optional): A function to handle the result of the action.
            feedback_handler (Callable[[Blackboard, Any], None], optional): A function to handle feedback from the action.
            callback_group (CallbackGroup, optional): The callback group for the action client.
            node (Node, optional): The ROS 2 node to use. If None, uses the default YasminNode.
            wait_timeout (float, optional): The maximum time to wait for the action server. Default is None (wait indefinitely).
            response_timeout (float, optional): The maximum time to wait for the action response. Default is None (wait indefinitely).
            maximum_retry (int, optional): Maximum retries of the action if it returns timeout. Default is 3.

        Raises:
            ValueError: If create_goal_handler is None.
        """

        ## Event used to wait for action completion.
        self._action_done_event: Event = Event()
        ## Shared pointer to the action result.
        self._action_result: Any = None
        ## Status of the action execution.
        self._action_status: GoalStatus = None
        ## Handle for the current goal.
        self._goal_handle: ClientGoalHandle = None
        ## Lock to manage access to the goal handle.
        self._goal_handle_lock: RLock = RLock()

        ## Handler function for creating goals.
        self._create_goal_handler: Callable[[Blackboard], Any] = create_goal_handler
        ## Handler function for processing results.
        self._result_handler: Callable[[Blackboard, Any], str] = result_handler
        ## Handler function for processing feedback.
        self._feedback_handler: Callable[[Blackboard, Any], None] = feedback_handler

        ## Maximum time to wait for the action server.
        self._wait_timeout: float = wait_timeout
        ## Timeout for the action response.
        self._response_timeout: float = response_timeout

        ## Maximum number of retries.
        self._maximum_retry: int = maximum_retry

        # Set outcomes
        outcomes = setup_outcomes(
            outcomes,
            {SUCCEED, ABORT, CANCEL},
            add_timeout=bool(self._wait_timeout or self._response_timeout),
        )

        self._node: Node = resolve_node(node)

        ## Name of the action to communicate with.
        self._action_name: str = action_name

        ## Action type for caching
        self._action_type: Type = action_type

        ## Shared pointer to the action client (reused from cache if available).
        self._action_client: ActionClient = ROSClientsCache.get_or_create_action_client(
            self._node,
            action_type,
            action_name,
            callback_group,
        )

        if not self._create_goal_handler:
            raise ValueError("create_goal_handler is needed")

        super().__init__(outcomes)

    def cancel_state(self) -> None:
        """
        Cancel the current action state.

        This function cancels the goal sent to the action server, if it exists.
        """
        with self._goal_handle_lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal()
        super().cancel_state()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the action and return the outcome.

        This function creates a goal using the provided goal handler, sends the
        goal to the action server, and waits for the result or feedback.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard used for communication.

        Returns:
            str: A string representing the outcome of the action execution.
                Possible outcomes include SUCCEED, ABORT, CANCEL, or TIMEOUT.
        """
        goal = self._create_goal_handler(blackboard)
        retry_count = 0

        yasmin.YASMIN_LOG_INFO(f"Waiting for action '{self._action_name}'")

        outcome = wait_with_retry(
            lambda: self._action_client.wait_for_server(self._wait_timeout),
            self._maximum_retry,
            f"Timeout reached, action '{self._action_name}' is not available",
            cancel_check=self.is_canceled,
        )
        if outcome is not None:
            return outcome

        if self.is_canceled():
            return CANCEL

        self._action_done_event.clear()

        yasmin.YASMIN_LOG_INFO(f"Sending goal to action '{self._action_name}'")

        def feedback_handler(feedback):
            if self._feedback_handler is not None:
                self._feedback_handler(blackboard, feedback.feedback)

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=feedback_handler
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        outcome = wait_with_retry(
            lambda: self._action_done_event.wait(self._response_timeout),
            self._maximum_retry,
            f"Timeout reached while waiting for response from "
            f"action '{self._action_name}'",
            cancel_check=self.is_canceled,
        )
        if outcome is not None:
            return outcome

        if self.is_canceled():
            return CANCEL

        status = self._action_status

        if status == GoalStatus.STATUS_CANCELED:
            return CANCEL

        elif status == GoalStatus.STATUS_ABORTED:
            return ABORT

        elif status == GoalStatus.STATUS_SUCCEEDED:
            if self._result_handler is not None:
                return self._result_handler(blackboard, self._action_result)

            return SUCCEED

        return ABORT

    def _goal_response_callback(self, future: Future) -> None:
        """
        Callback for handling the goal response.

        This method retrieves the goal handle and sets up the result callback.

        Args:
            future: The future object representing the result of the goal sending operation.
        """

        with self._goal_handle_lock:
            self._goal_handle: ClientGoalHandle = future.result()
            get_result_future: Future = self._goal_handle.get_result_async()
            get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future) -> None:
        """
        Callback for handling the result of the executed action.

        This method sets the action result and status, and signals that the action is done.

        Args:
            future: The future object representing the result of the action execution.
        """
        self._action_result: Any = future.result().result
        self._action_status: GoalStatus = future.result().status
        self._action_done_event.set()
