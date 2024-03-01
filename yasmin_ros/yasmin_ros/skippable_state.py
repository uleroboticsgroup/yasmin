from typing import List, Callable
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
        execute_handler: Callable,
        outcomes: List[str] = None,
    ) -> None:

        _outcomes = [SKIPPED]

        if outcomes:
            _outcomes = _outcomes + outcomes

        self._skipped = False
        self.__srv = node.create_service(
            Trigger, skip_srv_name, self._skip_state)

        self.__execute_handler = execute_handler

        super().__init__(_outcomes)

    def _skip_state(
        self,
        req: Trigger.Request,
        res: Trigger.Response
    ) -> Trigger.Response:
        self._skipped = True
        return Trigger.Response()

    def execute(self, blackboard: Blackboard) -> str:

        if self._skipped:
            return SKIPPED

        return self.__execute_handler(blackboard)