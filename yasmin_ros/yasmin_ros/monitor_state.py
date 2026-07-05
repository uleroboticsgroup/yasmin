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

from threading import Event
from typing import List, Set, Callable, Union, Type, Any

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL
from yasmin_ros.ros_state_utils import resolve_node, setup_outcomes, cancel_with_event


class MonitorState(State):
    """
    Template class to monitor a ROS 2 topic and process incoming messages.

    This class provides functionality to subscribe to a ROS 2 topic,
    execute a custom monitoring handler, and return specific outcomes
    based on the messages received.
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

        # Set outcomes
        outcomes = setup_outcomes(
            outcomes,
            {CANCEL},
            add_timeout=timeout is not None,
        )

        self._node = resolve_node(node)

        ## Name of the topic to monitor.
        self._topic_name: str = topic_name

        self._msg_type: Type = msg_type
        self._qos: Union[QoSProfile, int] = qos
        self._callback_group: CallbackGroup = callback_group

        self._sub = self._node.create_subscription(
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

            if self._timeout is not None and not timeout_flag:
                if retry_count >= self._maximum_retry:
                    return TIMEOUT

                retry_count += 1
                yasmin.YASMIN_LOG_WARN(
                    f"Timeout reached, topic '{self._topic_name}' is not available "
                    f"({retry_count}/{self._maximum_retry})"
                )

        yasmin.YASMIN_LOG_INFO(f"Processing msg from topic '{self._topic_name}'")
        outcome = self._monitor_handler(blackboard, self.msg_list.pop(0))

        return outcome

    def cancel_state(self) -> None:
        """
        Cancels the current monitor state.

        This method cancels the monitor if waiting for messages.
        """

        cancel_with_event(self._msg_event)
        super().cancel_state()
