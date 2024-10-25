# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import Dict, List
from rclpy.node import Node
from yasmin_msgs.msg import (
    State as StateMsg,
    StateMachine as StateMachineMsg,
    Transition as TransitionMsg,
)
from yasmin_ros.yasmin_node import YasminNode
from yasmin import StateMachine, State


class YasminViewerPub:

    _node: Node
    _fsm: StateMachine
    _fsm_name: str

    def __init__(
        self, fsm_name: str, fsm: StateMachine, rate: int = 4, node: Node = None
    ) -> None:

        self._fsm = fsm
        self._fsm_name = fsm_name

        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        self.pub = self._node.create_publisher(StateMachineMsg, "/fsm_viewer", 10)
        self._timer = self._node.create_timer(1 / rate, self._start_publisher)

    def parse_transitions(self, transitions: Dict[str, str]) -> List[TransitionMsg]:

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

        state_msg = StateMsg()

        state_msg.id = len(states_list)
        state_msg.parent = parent

        # state info
        state: State = state_info["state"]
        state_msg.name = state_name
        state_msg.transitions = self.parse_transitions(state_info["transitions"])
        state_msg.outcomes = state.get_outcomes()

        # state is a FSM
        state_msg.is_fsm = isinstance(state, StateMachine)

        # add state
        states_list.append(state_msg)

        # states of the FSM
        if state_msg.is_fsm:

            fsm: StateMachine = state

            states = fsm.get_states()

            for state_name in states:
                state_info = states[state_name]
                self.parse_state(state_name, state_info, states_list, state_msg.id)

            current_state = fsm.get_current_state()

            for child_state in states_list:
                if (
                    child_state.name == current_state
                    and child_state.parent == state_msg.id
                ):
                    state_msg.current_state = child_state.id
                    break

    def _start_publisher(self) -> None:

        states_list = []
        self.parse_state(
            self._fsm_name, {"state": self._fsm, "transitions": {}}, states_list
        )

        state_machine_msg = StateMachineMsg()
        state_machine_msg.states = states_list

        self.pub.publish(state_machine_msg)
