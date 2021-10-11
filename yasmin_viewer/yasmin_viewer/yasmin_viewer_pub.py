

from typing import Dict, List
from threading import Thread
import rclpy
from rclpy.node import Node
from yasmin_interfaces.msg import (
    State as StateMsg,
    StateInfo,
    StateMachine as StateMachineMsg,
    Structure,
    Transition
)
from yasmin import StateMachine, State


class YasminViewerPub:

    def __init__(self, node: Node, fsm_name: str, fsm: StateMachine):
        self.__fsm = fsm
        self.__fsm_name = fsm_name
        self.__node = node

        thread = Thread(target=self._start_publisher)
        thread.start()

    def parse_state_info(self, state_name: str, state: State) -> StateInfo:
        state_info_msg = StateInfo()

        for key in state["transitions"]:
            transition = Transition()
            transition.outcome = key
            transition.state = state["transitions"][key]
            state_info_msg.transitions.append(transition)

        for key in state["state"].get_outcomes():
            state_info_msg.outcomes.append(key)

        state_info_msg.state_name = state_name

        return state_info_msg

    def parse_states(self, states: Dict[str, str]) -> Structure:

        structure_msg = Structure()

        for state_n in states:

            state = states[state_n]
            state_o = state["state"]

            state_msg = StateMsg()
            state_info = self.parse_state_info(
                state_n, state)
            state_msg.state_info = state_info

            if isinstance(state_o, StateMachine):

                aux_structure_msg = self.parse_states(state_o.get_states())
                state_msg.is_fsm = True

                for state in aux_structure_msg.states:
                    state_msg.states.append(state.state_info)

                state_msg.current_state = state_o.get_current_state()

            structure_msg.states.append(state_msg)

        return structure_msg

    def parse_status(self, fsm: StateMachine) -> StateMachineMsg:

        states = fsm.get_states()
        status_msg = StateMachineMsg()

        structure_msg = self.parse_states(states)
        structure_msg.final_outcomes = fsm.get_outcomes()

        status_msg.fsm_structure = structure_msg
        status_msg.current_state = fsm.get_current_state()

        return status_msg

    def _start_publisher(self):
        publisher = self.__node.create_publisher(
            StateMachineMsg, "/fsm_viewer", 10)

        rate = self.__node.create_rate(4)

        while rclpy.ok():
            status_msg = self.parse_status(self.__fsm)
            status_msg.fsm_name = self.__fsm_name
            publisher.publish(status_msg)
            rate.sleep()
