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

import uuid
from threading import Thread, RLock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class YasminNode(Node):
    """
    A ROS 2 node for managing and handling YASMIN-based applications.

    YasminNode is a singleton class derived from Node and integrates
    custom functionalities for executing specific tasks in a ROS 2 environment.

    Attributes:
        _instance (YasminNode): The single instance of YasminNode.
        _lock (RLock): A reentrant lock for thread safety.
        _executor (MultiThreadedExecutor): Executor for managing multiple threads.
        _spin_thread (Thread): Thread for spinning the node.

    Raises:
        RuntimeError: Raised when attempting to instantiate the node more than once.
    """

    ## The single instance of YasminNode.
    _instance: "YasminNode" = None
    ## Lock to control access to the instance.
    _lock: RLock = RLock()

    @staticmethod
    def get_instance() -> "YasminNode":
        """
        Provides access to the singleton instance of YasminNode.

        This method ensures there is only one instance of YasminNode running.

        Returns:
            YasminNode: A reference to the YasminNode instance.

        Raises:
            RuntimeError: Raised if the creation of the instance fails.
        """
        with YasminNode._lock:
            if not rclpy.ok():
                rclpy.init()

            if YasminNode._instance is None:
                YasminNode._instance = YasminNode()

            return YasminNode._instance

    def __init__(self) -> None:
        """
        Default constructor. Initializes the node with a unique name.

        This constructor initializes the ROS 2 Node and
        starts a MultiThreadedExecutor for handling node callbacks.
        It raises a RuntimeError if an attempt is made to create a second instance
        of this Singleton class.

        Raises:
            RuntimeError: Raised when an attempt is made to create
            more than one instance of YasminNode.
        """
        if YasminNode._instance is not None:
            raise RuntimeError("This class is a Singleton")

        super().__init__(f"yasmin_{str(uuid.uuid4()).replace('-', '')[:16]}_node")

        ## Executor for managing node operations.
        self._executor: MultiThreadedExecutor = MultiThreadedExecutor()
        self._executor.add_node(self)

        ## Thread to execute the spinning of the node.
        self._spin_thread: Thread = Thread(target=self._executor.spin)
        self._spin_thread.start()
