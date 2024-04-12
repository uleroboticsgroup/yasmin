from typing import List, Callable, Type
from std_srvs.srv import SetBool, Trigger

from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SKIPPED


class SkippableState(State):

    def __init__(
        self,
        node: Node,
        skip_srv_name: str,
        srv_type: Type,
        execute_handler: Callable,
        outcomes: List[str] = None,
    ) -> None:

        _outcomes = [SKIPPED]

        if outcomes:
            _outcomes = _outcomes + outcomes

        self._skipped = False
        self.__srv = node.create_service(
            srv_type, skip_srv_name, self._skip_state)

        self.__execute_handler = execute_handler

        super().__init__(_outcomes)

    def _skip_state(
        self,
        req: Type.Request,
        res: Type.Response
    ) -> Type.Response:
        pass
    
    
    def execute(self, blackboard: Blackboard) -> str:

        if self._skipped:
            return SKIPPED

        return self.__execute_handler(blackboard)