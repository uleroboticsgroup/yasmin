
from threading import Thread, Lock
from typing import List, Dict, Optional

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class Concurrence(State):
    """
    Child class of a State that runs multiple other states in parallel threads.

    Attributes:
        _states (List[State]): A list of states that will be run in parallel by this state.
        _outcome_map (Dict[str, Dict[str, str]]): A dictionary correlating the outcomes of the concurrent states to
                                                  outcomes of this state.
        _default_outcome (str): A default outcome in case none of the correlations in _outcome_map are satisfied.
        _intermediate_outcomes (Dict[str, Optional[str]]): A temporary storage of the parallel states's outcomes.
        _mutex (Lock): A mutex to ensure thread safety of _intermediate_outcomes. 
    """

    def __init__(self, default_outcome: str, outcome_map: Dict[str, Dict[str, str]], states: List[State]) -> None:
        """
        Initializes the Concurrence instance.

        :param default_outcome: A default outcome in case none of the correlations in outcome_map are satisfied.
        :param outcome_map: A dictionary correlating the outcomes of the concurrent states to outcomes of this state.
        :param states: A list of states that will be run in parallel by this state.

        :raises ValueError: If either the provided outcomes set or states set are empty.
        """

        outcomes = list(set(outcome_map.keys()) | set([default_outcome]))

        super().__init__(outcomes)

        if not states:
            raise ValueError("There must be at least one state")

        self._states: List[State] = states
        self._outcome_map: Dict[str, Dict[str, str]] = outcome_map
        self._default_outcome: str = default_outcome

        self._intermediate_outcomes: Dict[str, Optional[str]] = {}
        for state in self._states:
            self._intermediate_outcomes[state.__str__()] = None

        self._mutex = Lock()

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the parallel behavior.

        :param blackboard: An instance of Blackboard that provides the context for execution.
        :return: The outcome of the execution as a string.
        """
        
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
            yasmin.YASMIN_LOG_WARN(f"More than one satisfied outcome after concurrent state execution.")        

        return satisfied_outcomes[0]

    def execute_and_save_state(self, state: State, blackboard: Blackboard) -> None:
        """
        Executes a state and saves its outcome to the intermediate map.

        :param state: A state to execute.
        :param blackboard: An instance of Blackboard that provides the context for execution.
        :return: None
        """
        outcome = state(blackboard)
        with self._mutex:
            self._intermediate_outcomes[state.__str__()] = outcome

    def cancel_state(self) -> None:
        """
        Cancels the execution of all states.
        """
        for state in self._states:
            state.cancel_state()

        super().cancel_state()

    def __str__(self) -> str:
        """
        Returns a string representation of the concurrence state, listing all states.

        Returns:
            str: A string representation of the state machine.
        """
        result = "Concurrence ["

        for i, state in enumerate(self._states):
            result += f"{state.__str__()}"

            if i < len(self._states) - 1:
                result += ", "

        result += "]"
        return result