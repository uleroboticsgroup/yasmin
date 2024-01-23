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


from typing import List, Callable, Type, Any
from yasmin import State
from yasmin import Blackboard
from simple_node import Node
from .basic_outcomes import SUCCEED, ABORT, CANCEL


class ActionState(State):

    def __init__(
        self,
        node: Node,
        action_type: Type,
        action_name: str,
        create_goal_handler: Callable,
        outcomes: List[str] = None,
        result_handler: Callable = None
    ) -> None:

        _outcomes = [SUCCEED, ABORT, CANCEL]

        if outcomes:
            _outcomes = _outcomes + outcomes

        self.__action_client = node.create_action_client(
            action_type, action_name)

        self.__create_goal_handler = create_goal_handler
        self.__result_handler = result_handler

        if not self.__create_goal_handler:
            raise Exception("create_goal_handler is needed")

        super().__init__(_outcomes)

    def _create_goal(self, blackboard: Blackboard) -> Any:
        return self.__create_goal_handler(blackboard)

    def cancel_state(self) -> None:
        self.__action_client.cancel_goal()
        super().cancel_state()

    def execute(self, blackboard: Blackboard) -> str:

        goal = self._create_goal(blackboard)
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_canceled():
            return CANCEL

        elif self.__action_client.is_aborted():
            return ABORT

        elif self.__action_client.is_succeeded():
            result = self.__action_client.get_result()

            if self.__result_handler is not None:
                outcome = self.__result_handler(blackboard, result)
                return outcome

            return SUCCEED

        return ABORT
