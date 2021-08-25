

from typing import List
from abc import ABC, abstractmethod
from .blackboard import Blackboard


class State(ABC):

    def __init__(self, outcomes: List[str]):
        self._outcomes = []
        self._canceled = False

        if outcomes:
            self._outcomes = outcomes
        else:
            raise Exception("There must be at least one outcome")

    def __call__(self, blackboard: Blackboard = None):
        self._canceled = False

        if blackboard is None:
            blackboard = Blackboard()

        outcome = self.execute(blackboard)

        if not outcome in self._outcomes:
            raise Exception("Outcome " + outcome + " does not belong")

        return outcome

    @abstractmethod
    def execute(self, blackboard: Blackboard) -> str:
        """ state execution """

    def __str__(self):
        return self.__class__.__name__

    def cancel_state(self):
        self._canceled = True

    def is_canceled(self):
        return self._canceled

    def get_outcomes(self) -> List[str]:
        return self._outcomes
