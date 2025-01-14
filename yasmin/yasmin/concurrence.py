
from threading import Thread, Lock
from typing import List, Dict, Optional

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class Concurrence(State):

    def __init__(self, default_outcome: str, outcome_map: Dict[str, Dict[str, str]], states: List[State]) -> None:

        outcomes = list(set(outcome_map.keys()) | set([default_outcome]))

        super().__init__(outcomes)

        self._states = states
        self._outcome_map = outcome_map
        self._default_outcome = default_outcome

        self._intermediate_outcomes: Dict[str, Optional[str]] = {}
        for state in self._states:
            self._intermediate_outcomes[state.__str__()] = None

        self._mutex = Lock()

    def execute(self, blackboard: Blackboard) -> str:

        state_threads = []

        for s in self._states:
            state_threads.append(Thread(target=self.execute_and_save_state, args=(s,blackboard,)))
            state_threads[-1].start()

        for t in state_threads:
            t.join()

        satisfied_outcomes = []
        for i, (outcome, requirements) in enumerate(self._outcome_map.items()):
            satisfied = True
            
            for state_name, state_outcome in requirements.items():
                satisfied = satisfied and (self._intermediate_outcomes[state_name] == state_outcome)

            if satisfied:
                satisfied_outcomes.append(outcome)

        if len(satisfied_outcomes) == 0:
            return self._default_outcome

        if len(satisfied_outcomes) > 1:
            yasmin.YASMIN_LOG_WARN(f"More than one satisfied outcome after concurrent state execution; returning the first one.")        

        return satisfied_outcomes[0]

    def execute_and_save_state(self, state: State, blackboard: Blackboard) -> None:
        outcome = state(blackboard)
        with self._mutex:
            self._intermediate_outcomes[state.__str__()] = outcome

    def cancel_state(self) -> None:
        for state in self._states:
            state.cancel_state()

        super().cancel_state()

    def __str__(self) -> str:
        """
        Returns a string representation of the concurrence state, listing all states.

        Returns:
            str: A string representation of the state machine.
        """
        result = "State Machine ["

        for i, state in enumerate(self._states):
            result += f"{state.__str__()}"

            if i < len(self._states) - 1:
                result += ", "

        result += "]"
        return result