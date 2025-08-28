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
    CbState is a subclass of State that encapsulates a callback function
    which can be executed in response to a specific state.

    Attributes:
        _cb (Callable): A callback function that defines the action to take
                        when the state is executed.
        _args (tuple): Positional arguments passed to the callback.
        _kwargs (dict): Keyword arguments passed to the callback.

    Parameters:
        outcomes (Set[str]): A set of possible outcomes for this state.
        cb (Callable): A callable that will be invoked when the state is executed.
    """

    def __init__(
        self, outcomes: Set[str], cb: Callable, *args: Any, **kwargs: Any
    ) -> None:
        """
        Initializes a new instance of CbState.

        Args:
            outcomes (Set[str]): A set of possible outcomes for this state.
            cb (Callable): A callable that defines the action to take when
                           the state is executed.
            *args (Any): Positional arguments for the callback.
            **kwargs (Any): Keyword arguments for the callback.

        Raises:
            TypeError: If 'outcomes' is not a set or 'cb' is not callable.
        """
        super().__init__(outcomes)

        ## A callable that will be invoked when the state is executed.
        self._cb: Callable = cb
        self._args = args
        self._kwargs = kwargs

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the callback function with the blackboard and provided arguments.

        Args:
            blackboard: The context in which the callback will be executed.
                        This is typically an object that holds the necessary
                        state or data for the callback.

        Returns:
            str: The result of the callback function execution.

        Raises:
            Exception: Propagates any exceptions raised by the callback function.
        """
        return self._cb(blackboard, *self._args, **self._kwargs)
