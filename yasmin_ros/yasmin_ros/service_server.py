from typing import List, Callable, Union, Type
from std_srvs.srv import SetBool, Trigger

from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SUCCEED, SKIPPED


class ServiceServer(State):

    def __init__(
        self,
        node: Node,
        srv_type: Type,
        srv_name: str,
        execute_handler: Callable,
        outcomes: List[str] = None,
    ) -> None:

        _outcomes = [SUCCEED, SKIPPED]

        if outcomes:
            _outcomes = _outcomes + outcomes
        self.__srv = node.create_service(
            srv_type, srv_name, self._srv_callback)

        self.__execute_handler = execute_handler

        super().__init__(_outcomes)

    def _srv_callback(
        self,
        req: Type.Request,
        res: Type.Response
    ) -> Type.Response:
        pass
    
    def execute(self, blackboard: Blackboard) -> str:
        return self.__execute_handler(blackboard)