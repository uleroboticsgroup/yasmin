# Copyright (C) 2023 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from typing import Set, Callable, Any

from yasmin.state import State
from yasmin.blackboard import Blackboard


class CbState(State):
    """
    Represents a state that executes a callback function.

    The CbState class inherits from the State class and is designed to
    execute a user-defined callback function, utilizing a shared pointer
    to a Blackboard object to obtain necessary data.

    Attributes:
        _cb (Callable): Pointer to the callback function to be executed.
        _args (tuple): Positional arguments passed to the callback.
        _kwargs (dict): Keyword arguments passed to the callback.
    """

    def __init__(
        self, outcomes: Set[str], cb: Callable, *args: Any, **kwargs: Any
    ) -> None:
        """
        Constructs a CbState object.

        Args:
            outcomes (Set[str]): A set of possible outcomes for this state.
            cb (Callable): A function pointer to the callback function that
                         will be executed when this state is activated.
            *args (Any): Positional arguments for the callback.
            **kwargs (Any): Keyword arguments for the callback.

        Raises:
            ValueError: If the outcomes set is empty.
        """
        super().__init__(outcomes)

        ## A callable that will be invoked when the state is executed.
        self._cb: Callable = cb
        self._args = args
        self._kwargs = kwargs

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the callback function.

        This function is called to execute the callback and retrieve
        the result. It may use the provided Blackboard for additional data.

        Args:
            blackboard (Blackboard): A shared pointer to the Blackboard object
                           used during execution.

        Returns:
            str: The result of the callback function execution as a string.

        Raises:
            RuntimeError: If the callback execution fails.
        """
        return self._cb(blackboard, *self._args, **self._kwargs)
