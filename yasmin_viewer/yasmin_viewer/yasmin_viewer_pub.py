# Copyright (C) 2023  Miguel Ángel González Santamarta
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

from typing import Dict, List
from rclpy.node import Node
import yasmin
from yasmin import StateMachine, State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_msgs.msg import (
    State as StateMsg,
    StateMachine as StateMachineMsg,
    Transition as TransitionMsg,
)


class YasminViewerPub:
    """
    A class to publish the state of a Finite State Machine (FSM) for visualization.

    Attributes:
        _node (Node): The ROS 2 node instance used for publishing.
        _fsm (StateMachine): The finite state machine to be published.
        _fsm_name (str): The name of the finite state machine.
        pub: The publisher for the state machine messages.
        _timer: A timer to periodically publish the FSM state.

    Methods:
        parse_transitions(transitions): Converts a dictionary of transitions to a list of TransitionMsg.
        parse_state(state_name, state_info, states_list, parent): Parses a state and its children recursively.
        _publish_data(): Publishes the current state of the FSM.
    """

    def __init__(
        self,
        fsm_name: str,
        fsm: StateMachine,
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

        if not fsm_name:
            raise ValueError("FSM name cannot be empty.")

        ## The finite state machine to be published.
        self._fsm: StateMachine = fsm

        ## The name of the finite state machine.
        self._fsm_name: str = fsm_name

        ## The ROS 2 node instance used for publishing.
        self._node = node

        if self._node is None:
            self._node: Node = YasminNode.get_instance()

        ## The publisher for the state machine messages.
        self.pub = self._node.create_publisher(StateMachineMsg, "/fsm_viewer", 10)

        ## A timer to periodically publish the FSM state.
        self._timer = self._node.create_timer(1 / rate, self._publish_data)

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
        state_info: Dict[str, str],
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

        # Check if the state is a FSM
        state_msg.is_fsm = isinstance(state, StateMachine)

        # Add state to the list
        states_list.append(state_msg)

        # Parse child states if this state is a FSM
        if state_msg.is_fsm:
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
            self.pub.publish(state_machine_msg)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f"Not publishing state machine '{self._fsm_name}' due to validation failure: {e}"
            )
