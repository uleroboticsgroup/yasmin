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
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros.ros_clients_cache import ROSClientsCache


class PublisherState(State):
    """
    Template class to publish ROS 2 messages.

    This class provides functionality to publish to a ROS 2 topic
    and create custom messages.

    Attributes:
        _node (Node): Shared pointer to the ROS 2 node.
        _pub (Publisher): Publisher to the ROS 2 topic.
        _topic_name (str): Name of the topic to publish to.
        _create_message_handler (Callable[[Blackboard], Any]): Callback handler to create messages.
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
        Construct a new PublisherState with ROS 2 node and specific QoS.

        Args:
            msg_type (Type): The type of message to be published.
            topic_name (str): The name of the topic to publish to.
            create_message_handler (Callable[[Blackboard], Any]): A callback handler to create messages.
            qos (Union[QoSProfile, int], optional): Quality of Service settings for the topic.
            callback_group (CallbackGroup, optional): The callback group for the publisher.
            node (Node, optional): The ROS 2 node to use. If None, a default node is created.
        """

        ## Callback handler to create messages.
        self._create_message_handler: Callable[[Blackboard], Any] = create_message_handler

        ## Shared pointer to the ROS 2 node.
        self._node: Node = node

        if self._node is None:
            self._node = YasminNode.get_instance()

        ## Name of the topic to publish to.
        self._topic_name: str = topic_name

        ## Publisher to the ROS 2 topic.
        self._pub: Publisher = ROSClientsCache.get_or_create_publisher(
            self._node,
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
        Execute the publishing operation.

        Args:
            blackboard (Blackboard): A shared pointer to the blackboard for data storage.

        Returns:
            str: A string outcome indicating the result of the publishing operation.
        """

        yasmin.YASMIN_LOG_DEBUG(f"Publishing to topic '{self._topic_name}'")
        msg = self._create_message_handler(blackboard)
        self._pub.publish(msg)
        return SUCCEED
