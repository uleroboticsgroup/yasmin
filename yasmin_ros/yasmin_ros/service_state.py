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

from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from typing import List, Callable, Type, Any

from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SUCCEED, ABORT

class ServiceState(State):

    def __init__(
        self,
        node: Node,
        srv_type: Type,
        srv_name: str,
        create_request_handler: Callable,
        outcomes: List[str] = None,
        response_handler: Callable = None,
        pub_topic_name: str = None
    ) -> None:

        _outcomes = [SUCCEED, ABORT]

        if outcomes:
            _outcomes = list(set(_outcomes + outcomes))

        self.__service_client = node.create_client(srv_type, srv_name)

        self.__create_request_handler = create_request_handler
        self.__response_handler = response_handler

        if pub_topic_name: 
            pubsub_callback_group = MutuallyExclusiveCallbackGroup()
            self.__pub_topic = node.create_publisher(String, pub_topic_name, 10,
                                                      callback_group=pubsub_callback_group)
        else: self.__pub_topic = None
            
        if not self.__create_request_handler:
            raise Exception("create_request_handler is needed")

        super().__init__(_outcomes)

    def publish_msg(self, msg: str):
        """
        Args:
            msg: message (string) to publish
        Raises:
            ros2 publish message to <pub_topic_name>
        """
        _msg = String()
        _msg.data = msg
        if self.__pub_topic: self.__pub_topic.publish(_msg)

    def _create_request(self, blackboard: Blackboard) -> Any:
        return self.__create_request_handler(blackboard)

    def execute(self, blackboard: Blackboard) -> str:

        request = self._create_request(blackboard)
        while not self.__service_client.service_is_ready():
            if self._is_canceled():
                return ABORT
        try:
            response =  self.__service_client.call_async(request)
            while True:
                if response.done():
                    break
                if self._is_canceled():
                    return ABORT
        except:
            return ABORT

        if self.__response_handler:
            outcome = self.__response_handler(blackboard, response.result())
            return outcome

        return SUCCEED

    def _is_canceled(self):
        if self.is_canceled():
            self._canceled = False
            return True
        return False