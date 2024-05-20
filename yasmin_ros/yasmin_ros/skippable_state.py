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
from std_srvs.srv import SetBool, Trigger

from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SKIPPED, ABORT, SUCCEED, WAITING


class SkippableState(State):

    def __init__(
        self,
        node: Node,
        skip_srv_name: str,
        execute_handler: Callable = None,
        srv_type: Type = Trigger,
        outcomes: List[str] = None,
        pub_topic_name: str = None
    ) -> None:

        _outcomes = [SKIPPED, ABORT, SUCCEED]

        if outcomes:
            _outcomes = list(set(_outcomes + outcomes))

        self._skipped = False
        self.__srv = node.create_service(
            srv_type, skip_srv_name, self._skip_state)

        self.__execute_handler = execute_handler

        if pub_topic_name: 
            pubsub_callback_group = MutuallyExclusiveCallbackGroup()
            self.__pub_topic = node.create_publisher(String, pub_topic_name, 10,
                                                      callback_group=pubsub_callback_group)
        else: self.__pub_topic = None

        super().__init__(_outcomes)

    def _skip_state(
        self,
        req: Trigger.Request,
        res: Trigger.Response
    ) -> Trigger.Response:
        self._skipped = True
        return Trigger.Response()
    
    def execute(self, blackboard: Blackboard) -> str:
        while True:
            if self.__execute_handler: 
                outcome = self.__execute_handler(blackboard)
            if self._skipped:
                self._skipped = False
                return SKIPPED
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