

from typing import Dict, List
from threading import Thread
import rclpy
from rclpy.node import Node
from yasmin_interfaces.msg import (
    State as StateMsg,
    StateInfo,
    Status,
    Structure,
    Transition
)
from yasmin import StateMachine, State


class YasminViewerPub:

    def __init__(self, node: Node, fsm_name: str, fsm: StateMachine):
        self.__fsm = fsm
        self.__fsm_name = fsm_name
        self.__node = node

        thread = Thread(target=self.start_client)
        thread.start()

    def parse_state_info(self, name: str, state: Dict[str, str]) -> StateInfo:
        msg = StateInfo()

        for key in state["transitions"]:
            transition = Transition()
            transition.outcome = key
            transition.state = state["transitions"][key]
            msg.transitions.append(transition)

        for key in state["state"].get_outcomes():
            msg.outcomes.append(key)

        msg.state_name = name

        return msg

    def parse_states(self, states: List[State]) -> Structure:

        fsm_structure = Structure()

        for state_n in states:

            state = states[state_n]
            state_o = state["state"]

            state_msg = StateMsg()
            state_info = self.parse_state_info(
                state_n, state)
            state_msg.state = state_info

            if isinstance(state_o, StateMachine):

                structure_msg = self.parse_states(state_o.get_states())
                state_msg.is_fsm = True

                for state in structure_msg.states:
                    state_msg.states.append(state.state)

                state_msg.current_state = state_o.get_current_state()

            fsm_structure.states.append(state_msg)

        return fsm_structure

    def parse_status(self, fsm: StateMachine) -> Status:

        states = fsm.get_states()
        status_msg = Status()

        structure_msg = self.parse_states(states)
        structure_msg.final_outcomes = fsm.get_outcomes()

        status_msg.fsm_structure = structure_msg
        status_msg.current_state = fsm.get_current_state()

        return status_msg

    def start_client(self):
        publisher = self.__node.create_publisher(Status, "/fsm_viewer", 10)

        rate = self.__node.create_rate(4)

        while rclpy.ok():
            status_msg = self.parse_status(self.__fsm)
            status_msg.fsm_name = self.__fsm_name
            publisher.publish(status_msg)
            rate.sleep()
