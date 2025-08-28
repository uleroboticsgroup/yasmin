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
from threading import RLock, Event

from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup
from action_msgs.msg import GoalStatus

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT


class ActionState(State):
    """
    Represents a state that interacts with a ROS 2 action server.

    This class encapsulates the functionality needed to manage
    the sending and receiving of goals to and from an action server.

    Attributes:
        _node (Node): The ROS 2 node instance used to communicate with the action server.
        _action_name (str): The name of the action to be performed.
        _action_client (ActionClient): The action client used to send goals.
        _action_done_event (Event): Event used to wait for action completion.
        _action_result (Any): The result returned by the action server.
        _action_status (GoalStatus): The status of the action execution.
        _goal_handle (ClientGoalHandle): Handle for the goal sent to the action server.
        _goal_handle_lock (RLock): Lock to manage access to the goal handle.
        _create_goal_handler (Callable[[Blackboard], Any]): Function that creates the goal to send.
        _result_handler (Callable[[Blackboard, Any], str]): Function to handle the result from the action server.
        _feedback_handler (Callable[[Blackboard, Any], None]): Function to handle feedback from the action server.
        _timeout (float): Timeout duration for waiting for the action server.
    """

    def __init__(
        self,
        action_type: Type,
        action_name: str,
        create_goal_handler: Callable,
        outcomes: Set[str] = None,
        result_handler: Callable = None,
        feedback_handler: Callable = None,
        callback_group: CallbackGroup = None,
        node: Node = None,
        timeout: float = None,
    ) -> None:
        """
        Initializes the ActionState instance.

        This constructor sets up the action client and prepares to handle goals.

        Parameters:
            action_type (Type): The type of the action to be executed.
            action_name (str): The name of the action to be executed.
            create_goal_handler (Callable[[Blackboard], Any])): A function that generates the goal.
            outcomes (Set[str], optional): Additional outcomes that this state can return.
            result_handler (Callable[[Blackboard, Any], str], optional): A function to process the result of the action.
            feedback_handler (Callable[[Blackboard, Any], None], optional): A function to process feedback from the action.
            callback_group (CallbackGroup, optional): The callback group for the action client.
            node (Node, optional): The ROS 2 node to use. If None, uses the default YasminNode.
            timeout (float, optional): Timeout duration for waiting for the action server.

        Raises:
            ValueError: If create_goal_handler is None.
        """

        ## Event used to wait for action completion.
        self._action_done_event: Event = Event()
        ## The result returned by the action server.
        self._action_result: Any = None
        ## The status of the action execution.
        self._action_status: GoalStatus = None
        ## Handle for the goal sent to the action server.
        self._goal_handle: ClientGoalHandle = None
        ## Lock to manage access to the goal handle.
        self._goal_handle_lock: RLock = RLock()

        ## Function that creates the goal to send.
        self._create_goal_handler: Callable[[Blackboard], Any] = create_goal_handler
        ## Function to handle the result from the action server.
        self._result_handler: Callable[[Blackboard, Any], str] = result_handler
        ## Function to handle feedback from the action server.
        self._feedback_handler: Callable[[Blackboard, Any], None] = feedback_handler

        ## Timeout duration for waiting for the action server.
        self._timeout: float = timeout

        _outcomes = [SUCCEED, ABORT, CANCEL]

        if self._timeout:
            _outcomes.append(TIMEOUT)

        if outcomes:
            _outcomes = _outcomes + outcomes

        ## The ROS 2 node instance used to communicate with the action server.
        self._node: Node = node

        if self._node is None:
            self._node: Node = YasminNode.get_instance()

        ## The name of the action to be performed.
        self._action_name: str = action_name

        ## The action client used to send goals.
        self._action_client: ActionClient = ActionClient(
            self._node,
            action_type,
            action_name,
            callback_group=callback_group,
        )

        if not self._create_goal_handler:
            raise ValueError("create_goal_handler is needed")

        super().__init__(_outcomes)

    def cancel_state(self) -> None:
        """
        Cancels the current action state.

        This method cancels the goal sent to the action server, if it exists.
        """
        with self._goal_handle_lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal()

        super().cancel_state()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the action state by sending a goal to the action server.

        This method waits for the action server to be available, sends the goal,
        and waits for the action to complete, handling feedback and results.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for state management.

        Returns:
            str: The outcome of the action execution (e.g., SUCCEED, ABORT, CANCEL, TIMEOUT).

        Raises:
            Exception: Raises an exception if any error occurs during action execution.
        """
        goal = self._create_goal_handler(blackboard)

        yasmin.YASMIN_LOG_INFO(f"Waiting for action '{self._action_name}'")
        act_available = self._action_client.wait_for_server(self._timeout)

        if not act_available:
            yasmin.YASMIN_LOG_WARN(
                f"Timeout reached, action '{self._action_name}' is not available"
            )
            return TIMEOUT

        self._action_done_event.clear()

        yasmin.YASMIN_LOG_INFO(f"Sending goal to action '{self._action_name}'")

        def feedback_handler(feedback):
            if self._feedback_handler is not None:
                self._feedback_handler(blackboard, feedback.feedback)

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=feedback_handler
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        # Wait for action to be done
        self._action_done_event.wait()
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
        Callback to handle the response from sending a goal.

        This method retrieves the goal handle and sets up the result callback.

        Parameters:
            future: The future object representing the result of the goal sending operation.
        """
        with self._goal_handle_lock:
            self._goal_handle: ClientGoalHandle = future.result()
            get_result_future = self._goal_handle.get_result_async()
            get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future) -> None:
        """
        Callback to handle the result of the executed action.

        This method sets the action result and status, and signals that the action is done.

        Parameters:
            future: The future object representing the result of the action execution.
        """
        self._action_result: Any = future.result().result
        self._action_status: GoalStatus = future.result().status
        self._action_done_event.set()
