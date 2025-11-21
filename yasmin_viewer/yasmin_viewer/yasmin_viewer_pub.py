# Copyright (C) 2023 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from typing import Any, Dict, List
from rclpy.node import Node
import yasmin
from yasmin import StateMachine, State, Concurrence
from yasmin_ros.yasmin_node import YasminNode
from yasmin_msgs.msg import (
    State as StateMsg,
    StateMachine as StateMachineMsg,
    Transition as TransitionMsg,
)


class YasminViewerPub(object):
    """
    A class to publish the state of a Finite State Machine (FSM) for visualization.
    """

    def __init__(
        self,
        fsm: StateMachine,
        fsm_name: str = "",
        rate: int = 4,
        node: Node = None,
    ) -> None:
        """
        Initializes the YasminViewerPub instance.

        Args:
            fsm_name (str): The name of the FSM.
            fsm (StateMachine): The FSM instance to be published.
            rate (int): The rate in Hz at which to publish updates. Defaults to 4.
            node (Node, optional): A custom Node instance. If None, a new YasminNode is created.

        Raises:
            ValueError: If fsm_name is empty.
        """

        ## The ROS 2 node instance used for publishing.
        self._node = node

        if self._node is None:
            self._node: Node = YasminNode.get_instance()

        ## The name of the finite state machine.
        self._fsm_name: str = fsm_name

        if not self._fsm_name and not fsm.get_name():
            self._fsm_name = "Unnamed_FSM"
        elif not self._fsm_name:
            self._fsm_name = fsm.get_name()

        ## The finite state machine to be published.
        self._fsm: StateMachine = fsm

        ## The publisher for the state machine messages.
        self._pub = self._node.create_publisher(StateMachineMsg, "/fsm_viewer", 10)

        ## A timer to periodically publish the FSM state.
        self._timer = self._node.create_timer(1 / rate, self._publish_data)

    def cleanup(self) -> None:
        """
        Cleans up resources used by the YasminViewerPub instance.
        """
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
        del self._timer
        if self._pub is not None:
            self._node.destroy_publisher(self._pub)
        del self._pub
        self._fsm = None

    def parse_transitions(self, transitions: Dict[str, str]) -> List[TransitionMsg]:
        """
        Converts a dictionary of transitions into a list of TransitionMsg.

        Args:
            transitions (Dict[str, str]): A dictionary where keys are outcome names and values are state names.

        Returns:
            List[TransitionMsg]: A list of TransitionMsg representing the FSM transitions.
        """
        transitions_list = []

        for key in transitions:
            transition = TransitionMsg()
            transition.outcome = key
            transition.state = transitions[key]
            transitions_list.append(transition)

        return transitions_list

    def parse_state(
        self,
        state_name: str,
        state_info: Dict[str, Any],
        states_list: List[StateMsg],
        parent: int = -1,
    ) -> None:
        """
        Recursively parses a state and its transitions, adding the resulting StateMsg to the states list.

        Args:
            state_name (str): The name of the state.
            state_info (Dict[str, str]): Information about the state, including its transitions.
            states_list (List[StateMsg]): A list to which the parsed StateMsg will be appended.
            parent (int, optional): The ID of the parent state. Defaults to -1.

        Returns:
            None
        """
        state_msg = StateMsg()

        state_msg.id = len(states_list)
        state_msg.parent = parent

        # Extract state information
        state: State = state_info["state"]
        state_msg.name = state_name
        state_msg.transitions = self.parse_transitions(state_info["transitions"])
        state_msg.outcomes = state.get_outcomes()

        # Check if the state is a FSM or Concurrence
        state_msg.is_fsm = isinstance(state, StateMachine) or isinstance(
            state, Concurrence
        )

        # Add state to the list
        states_list.append(state_msg)

        # Parse child states if this state is a FSM
        if isinstance(state, StateMachine):
            fsm: StateMachine = state
            states = fsm.get_states()

            for child_state_name in states:
                child_state_info = states[child_state_name]
                self.parse_state(
                    child_state_name, child_state_info, states_list, state_msg.id
                )

            current_state = fsm.get_current_state()

            # Find the current state among the child states
            for child_state in states_list:
                if (
                    child_state.name == current_state
                    and child_state.parent == state_msg.id
                ):
                    state_msg.current_state = child_state.id
                    break

        # Parse child states if this state is a Concurrence
        elif isinstance(state, Concurrence):
            concurrence: Concurrence = state
            concurrent_states = concurrence.get_states()
            transitions = self.parse_concurrence_transitions(concurrence)
            state_msg.current_state = -2

            for child_state_name, child_state in concurrent_states.items():
                child_state_info = {
                    "state": child_state,
                    "transitions": {},
                }
                self.parse_state(
                    child_state_name, child_state_info, states_list, state_msg.id
                )
                states_list[-1].transitions = transitions[child_state_name]

                # Check if the child_state outcomes are in the transitions
                for outcome in child_state.get_outcomes():
                    if outcome not in [t.outcome for t in states_list[-1].transitions]:
                        # If not, add a transition to the default outcome of the concurrence
                        msg = TransitionMsg()
                        msg.outcome = outcome
                        msg.state = concurrence.get_default_outcome()
                        states_list[-1].transitions.append(msg)

    def parse_concurrence_transitions(
        self, concurrence: Concurrence
    ) -> Dict[str, Dict[str, str]]:
        """
        Converts a concurrence outcome map into transition-like information for visualization.

        In concurrence, transitions are based on outcome combinations rather than direct state-to-state transitions.
        This method creates pseudo-transitions that represent the outcome mapping logic.

        Args:
            concurrence (Concurrence): The concurrence state to parse transitions from.

        Returns:
            List[TransitionMsg]: A list of TransitionMsg representing the concurrence outcome mappings.
        """
        transitions = {}
        outcome_map = concurrence.get_outcome_map()

        # Add transitions for each outcome in the outcome map
        for outcome, requirements in outcome_map.items():
            for state_name, state_outcome in requirements.items():

                if state_name not in transitions:
                    transitions[state_name] = []
                msg = TransitionMsg()
                msg.outcome = state_outcome
                msg.state = outcome
                transitions[state_name].append(msg)

        return transitions

    def _publish_data(self) -> None:
        """
        Publishes the current state of the FSM.

        This method validates the FSM, gathers its state data, and publishes it.
        If validation fails, an error message is logged.

        Returns:
            None

        Raises:
            Exception: If the FSM validation fails, an error is logged and the function exits without publishing.
        """
        try:
            self._fsm.validate()

            states_list = []
            self.parse_state(
                self._fsm_name,
                {"state": self._fsm, "transitions": {}},
                states_list,
            )

            state_machine_msg = StateMachineMsg()
            state_machine_msg.states = states_list

            yasmin.YASMIN_LOG_DEBUG(
                f"Publishing data of state machine '{self._fsm_name}'"
            )
            self._pub.publish(state_machine_msg)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f"Not publishing state machine '{self._fsm_name}' due to validation failure: {e}"
            )
