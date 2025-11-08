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
from typing import List, Set, Callable, Union, Type, Any

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL


class MonitorState(State):
    """
    Template class to monitor a ROS 2 topic and process incoming messages.

    This class provides functionality to subscribe to a ROS 2 topic,
    execute a custom monitoring handler, and return specific outcomes
    based on the messages received.

    Attributes:
        _node (Node): Shared pointer to the ROS 2 node.
        _sub (Subscription): Subscription to the ROS 2 topic.
        _monitor_handler (Callable[[Blackboard, Any], str]): Function to handle incoming messages.
        _topic_name (str): Name of the topic to monitor.
        msg_list (List[Any]): List to store queued messages.
        msg_queue (int): Maximum number of messages to queue.
        _timeout (int): Timeout in seconds for message reception.
        _maximum_retry (int): Maximum number of retries.
        _msg_event (Event): Event for message reception.
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
        maximum_retry: int = 3,
    ) -> None:
        """
        Construct a new MonitorState with specific QoS, message queue, and timeout.

        Args:
            msg_type (Type): The type of message to be monitored.
            topic_name (str): The name of the topic to monitor.
            outcomes (Set[str]): A set of possible outcomes for this state.
            monitor_handler (Callable[[Blackboard, Any], str]): A callback handler to process incoming messages.
            qos (Union[QoSProfile, int], optional): Quality of Service settings for the topic.
            callback_group (CallbackGroup, optional): The callback group for the subscription.
            msg_queue (int, optional): The maximum number of messages to queue.
            node (Node, optional): The ROS 2 node to use. If None, a default node is created.
            timeout (int, optional): The time in seconds to wait for messages before timing out.
            maximum_retry (int, optional): Maximum retries of the monitor if it returns timeout. Default is 3.
        """

        ## Function to handle incoming messages.
        self._monitor_handler: Callable[[Blackboard, Any], str] = monitor_handler
        ## List to store queued messages.
        self.msg_list: List[Any] = []
        ## Maximum number of messages to queue.
        self.msg_queue: int = msg_queue
        ## Event for message reception.
        self._msg_event: Event = Event()

        ## Timeout in seconds for message reception.
        self._timeout: int = timeout

        if timeout is not None:
            outcomes = [TIMEOUT] + outcomes
        outcomes = [CANCEL] + outcomes

        ## Shared pointer to the ROS 2 node.
        self._node: Node = node

        if self._node is None:
            self._node = YasminNode.get_instance()

        ## Name of the topic to monitor.
        self._topic_name: str = topic_name

        ## Subscription to the ROS 2 topic.
        self._sub: Subscription = None
        self._msg_type: Type = msg_type
        self._qos: Union[QoSProfile, int] = qos
        self._callback_group: CallbackGroup = callback_group

        ## Subscription to the ROS 2 topic.
        self._sub: Subscription = self._node.create_subscription(
            msg_type,
            topic_name,
            self.__callback,
            qos,
            callback_group=callback_group,
        )

        ## Maximum number of retries.
        self._maximum_retry: int = maximum_retry

        super().__init__(outcomes)

    def __callback(self, msg: Any) -> None:
        """
        Callback function for receiving messages from the subscribed topic.

        Adds the message to msg_list maintaining a maximum queue size of msg_queue.

        Args:
            msg: The message received from the topic.
        """
        self.msg_list.append(msg)

        if len(self.msg_list) > self.msg_queue:
            self.msg_list.pop(0)

        self._msg_event.set()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Execute the monitoring operation and process the first received message.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard for data storage.

        Returns:
            str: A string outcome indicating the result of the monitoring operation.
        """

        # Wait until a message is received or timeout is reached
        self._msg_event.clear()
        retry_count = 0

        while not self.msg_list:
            timeout_flag = self._msg_event.wait(self._timeout)

            if self.is_canceled():
                return CANCEL

            while self._timeout is not None and not timeout_flag:
                yasmin.YASMIN_LOG_WARN(
                    f"Timeout reached, topic '{self._topic_name}' is not available"
                )

                if retry_count < self._maximum_retry:
                    retry_count += 1
                    yasmin.YASMIN_LOG_WARN(
                        f"Retrying to wait for topic '{self._topic_name}' ({retry_count}/{self._maximum_retry})"
                    )
                else:
                    return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f"Processing msg from topic '{self._topic_name}'")
        outcome = self._monitor_handler(blackboard, self.msg_list.pop(0))

        return outcome

    def cancel_state(self) -> None:
        """
        Cancels the current monitor state.

        This method cancels the monitor if waiting for messages.
        """

        self._msg_event.set()
        super().cancel_state()
