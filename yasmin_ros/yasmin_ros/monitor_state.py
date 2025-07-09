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

from typing import List, Set, Callable, Union, Type, Any
from threading import Event

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL


class MonitorState(State):
    """
    MonitorState is a state that monitors messages from a ROS topic.

    Attributes:
        _node (Node): The ROS 2 node instance used for subscriptions.
        _sub (Subscription): Subscription object for the specified topic.
        _monitor_handler (Callable[[Blackboard, Any], None]): Function to handle incoming messages.
        _topic_name (str): The name of the topic to monitor.
        msg_list (List[Any]): A set of messages received from the topic.
        msg_queue (int): The maximum number of messages to retain.
        time_to_wait (float): Time to wait between checks for messages.
        _timeout (int): Timeout duration for monitoring.

    Exceptions:
        None raised directly; timeout is managed via outcome handling.
    """

    def __init__(
        self,
        msg_type: Type,
        topic_name: str,
        outcomes: Set[str],
        monitor_handler: Callable,
        qos: Union[QoSProfile, int] = 10,
        callback_group: CallbackGroup = None,
        msg_queue: int = 10,
        node: Node = None,
        timeout: int = None,
    ) -> None:
        """
        Initializes the MonitorState.

        Parameters:
            msg_type (Type): The type of message to be monitored.
            topic_name (str): The name of the topic to subscribe to.
            outcomes (Set[str]): The set of possible outcomes from the state.
            monitor_handler (Callable[[Blackboard, Any], None]): The function to call with the received messages.
            qos (Union[QoSProfile, int], optional): Quality of Service profile or depth.
            callback_group (CallbackGroup, optional): The callback group for the subscription.
            msg_queue (int, optional): Maximum number of messages to store. Default is 10.
            node (Node, optional): The ROS node to use. If None, a default node is created.
            timeout (int, optional): Timeout in seconds for monitoring before giving up.

        Returns:
            None
        """

        ## Function to handle incoming messages.
        self._monitor_handler: Callable[[Blackboard, Any], None] = monitor_handler
        ## A set of messages received from the topic.
        self.msg_list: List[Any] = []
        ## The maximum number of messages to retain.
        self.msg_queue: int = msg_queue
        ## Time to wait between checks for messages.
        self._msg_event: Event = Event()

        ## Timeout duration for monitoring.
        self._timeout: int = timeout

        if timeout is not None:
            outcomes = [TIMEOUT] + outcomes
        outcomes = [CANCEL] + outcomes

        ## The ROS 2 node instance used for subscriptions.
        self._node: Node = node

        if self._node is None:
            self._node = YasminNode.get_instance()

        ## The name of the topic to monitor.
        self._topic_name: str = topic_name

        ## Subscription object for the specified topic.
        self._sub: Subscription = None
        self._msg_type: Type = msg_type
        self._qos: Union[QoSProfile, int] = qos
        self._callback_group: CallbackGroup = callback_group

        ## Subscription object for the specified topic.
        self._sub: Subscription = self._node.create_subscription(
            self._msg_type,
            self._topic_name,
            self.__callback,
            self._qos,
            callback_group=self._callback_group,
        )

        super().__init__(outcomes)

    def __callback(self, msg: Any) -> None:
        """
        Callback function that handles incoming messages.

        This method is called when a new message is received on the monitored topic.

        Parameters:
            msg: The message received from the topic.

        Returns:
            None
        """
        self.msg_list.append(msg)

        if len(self.msg_list) > self.msg_queue:
            self.msg_list.pop(0)

        self._msg_event.set()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the monitoring state.

        This method waits for messages from the monitored topic and processes them using
        the specified monitor handler.

        Parameters:
            blackboard (Blackboard): The blackboard instance that holds shared data.

        Returns:
            str: The outcome of the monitoring process. Returns TIMEOUT if the monitoring
            time exceeds the specified timeout.

        Exceptions:
            None raised directly; timeouts are handled gracefully.
        """

        # Wait until a message is received or timeout is reached
        self._msg_event.clear()

        while not self.msg_list:
            flag = self._msg_event.wait(self._timeout)

            if self.is_canceled():
                return CANCEL

            if self._timeout is not None and not flag:
                yasmin.YASMIN_LOG_WARN(
                    f"Timeout reached, topic '{self._topic_name}' is not available"
                )
                return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f"Processing msg from topic '{self._topic_name}'")
        outcome = self._monitor_handler(blackboard, self.msg_list.pop(0))

        return outcome

    def cancel_state(self) -> None:
        """
        Cancels the current monitor state.

        This method cancels the monitor if waiting for messages.
        """

        super().cancel_state()
        self._msg_event.set()
