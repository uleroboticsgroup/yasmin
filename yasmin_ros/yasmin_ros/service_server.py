from typing import List, Callable, Union, Type
from std_srvs.srv import SetBool, Trigger

from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SUCCEED, SKIPPED, CANCEL, ABORT
from abc import ABC, abstractmethod

class ServiceServer(State):

    def __init__(
        self,
        node: Node,
        srv_name: str,
        execute_handler: Callable,
        srv_type: Type = SetBool,
        outcomes: List[str] = None,
    ) -> None:

        _outcomes = [SUCCEED, CANCEL, ABORT]

        if outcomes:
            _outcomes = list(set(_outcomes + outcomes))
        self.__srv_server = node.create_service(
            srv_type, srv_name, self._srv_callback)

        self.__execute_handler = execute_handler

        super().__init__(_outcomes)
        print(self.__str__)

    @abstractmethod
    def _srv_callback(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        """ state service callback """

    def execute(self, blackboard: Blackboard) -> str:
        if self._is_canceled(): return ABORT
        return self.__execute_handler(blackboard)
    
    def _is_canceled(self):
        if self.is_canceled():
            self._canceled = False
            return True
        return False