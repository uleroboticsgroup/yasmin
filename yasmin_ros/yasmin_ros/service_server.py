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

from abc import ABCMeta, abstractmethod
from typing import List, Callable, Union, Type
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

from rclpy.node import Node
from rclpy.service import Service

from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT, CANCEL


class ServiceServer(State):

    _node: Node
    _srv_name: str
    _service_server: Service
    _create_request_handler: Callable
    _response_handler: Callable
    _timeout: float

    def __init__(
        self,
        srv_name: str,
        execute_handler: Callable,
        srv_type: Type = Trigger,
        outcomes: List[str] = None,
        node: Node = None,
        timeout: float = None,
        pub_topic_name: str = None
    ) -> None:
        
        self._srv_name = srv_name

        _outcomes = [SUCCEED, CANCEL, ABORT]

        self._timeout = timeout
        if self._timeout:
            _outcomes.append(TIMEOUT)
            
        if outcomes:
            _outcomes = _outcomes + outcomes

        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        self._service_server = self._node.create_service(
            srv_type, srv_name, self._srv_callback)
        
        if pub_topic_name: 
            self.__pub_topic = node.create_publisher(String, pub_topic_name, 10)

        else: self.__pub_topic = None

        self._execute_handler = execute_handler

        super().__init__(_outcomes)
        print(self.__str__)

    @abstractmethod
    def _srv_callback(
        self,
        req: Trigger.Request,
        res: Trigger.Response
    ) -> Trigger.Response:
        """ state service callback """

    def execute(self, blackboard: Blackboard) -> str:
        while True:
            if self._execute_handler: 
                outcome = self._execute_handler(blackboard)
            if self._is_canceled(): 
                return ABORT
            if outcome != WAITING: break
        return outcome
    
    def _is_canceled(self):
        if self.is_canceled():
            self._canceled = False
            return True
        return False
    
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