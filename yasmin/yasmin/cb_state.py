

from typing import List, Callable
from .state import State


class CbState(State):

    def __init__(self, outcomes: List[str], cb: Callable):

        super().__init__(outcomes)
        self._cb = cb

    def execute(self, blackboard):
        return self._cb(blackboard)
