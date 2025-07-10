# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import Callable, Union, Type, Any

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.callback_groups import CallbackGroup

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED


class PublisherState(State):
    """
    PublisherState is a state that publishes messages in a ROS topic.

    Attributes:
        _node (Node): The ROS 2 node instance used for subscriptions.
        _pub (Publisher): Publisher object for the specified topic.
        _create_message_handler (Callable[[Blackboard], Any]): Function to create new messages.
        _topic_name (str): The name of the topic to monitor.
    """

    def __init__(
        self,
        msg_type: Type,
        topic_name: str,
        create_message_handler: Callable,
        qos: Union[QoSProfile, int] = 10,
        callback_group: CallbackGroup = None,
        node: Node = None,
    ) -> None:
        """
        Initializes the MonitorState.

        Parameters:
            msg_type (Type): The type of message to be monitored.
            topic_name (str): The name of the topic to subscribe to.
            create_message_handler (Callable[[Blackboard], Any]): The function to call with the received messages.
            qos (Union[QoSProfile, int], optional): Quality of Service profile or depth.
            callback_group (CallbackGroup, optional): The callback group for the subscription.
            node (Node, optional): The ROS node to use. If None, a default node is created.

        Returns:
            None
        """

        ## Function to handle incoming messages.
        self._create_message_handler: Callable[[Blackboard], Any] = create_message_handler

        ## The ROS 2 node instance used for subscriptions.
        self._node: Node = node

        if self._node is None:
            self._node = YasminNode.get_instance()

        ## The name of the topic to monitor.
        self._topic_name: str = topic_name

        ## Publisher object for the specified topic.
        self._pub: Publisher = self._node.create_publisher(
            msg_type,
            topic_name,
            qos,
            callback_group=callback_group,
        )

        if not self._create_message_handler:
            raise ValueError("create_message_handler is needed")

        super().__init__([SUCCEED])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the publisher state.

        This method publishes messages in a topic.

        Parameters:
            blackboard (Blackboard): The blackboard instance that holds shared data.

        Returns:
            str: The outcome of the monitoring process. Returns TIMEOUT if the monitoring
            time exceeds the specified timeout.
        """

        yasmin.YASMIN_LOG_DEBUG(f"Publishing to topic '{self._topic_name}'")
        msg = self._create_message_handler(blackboard)
        self._pub.publish(msg)
        return SUCCEED
