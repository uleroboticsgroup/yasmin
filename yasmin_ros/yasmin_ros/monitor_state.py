
import time
from typing import List, Callable, Union

from rclpy.qos import QoSProfile

from yasmin import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import CANCEL
from simple_node import Node


class MonitorState(State):

    def __init__(self,
                 node: Node,
                 msg_type,
                 topic_name: str,
                 outcomes: List[str],
                 monitor_handler: Callable,
                 qos: Union[QoSProfile, int] = 10,
                 msg_queue: int = 10,
                 timeout: int = None) -> None:

        if not timeout is None:
            outcomes = [CANCEL] + outcomes
        super().__init__(outcomes)

        self.monitor_handler = monitor_handler
        self.topic_name = topic_name
        self.msg_list = []
        self.msg_queue = msg_queue
        self.timeout = timeout
        self.time_to_wait = 0.001
        self.monitoring = False

        self._sub = node.create_subscription(
            msg_type, topic_name, self.__callback, qos)

    def __callback(self, msg) -> None:

        if self.monitoring:
            self.msg_list.append(msg)

            if len(self.msg_list) >= self.msg_queue:
                self.msg_list.pop(0)

    def execute(self, blackboard: Blackboard) -> str:

        elapsed_time = 0
        self.msg_list = []
        self.monitoring = True

        while len(self.msg_list) == 0:
            time.sleep(self.time_to_wait)

            if not self.timeout is None:

                if elapsed_time >= self.timeout:
                    self.monitoring = False
                    return CANCEL

                elapsed_time += self.time_to_wait

        blackboard[self.topic_name + "_msg"] = self.msg_list.pop(0)

        outcome = self.monitor_handler(blackboard)
        self.monitoring = False
        return outcome
