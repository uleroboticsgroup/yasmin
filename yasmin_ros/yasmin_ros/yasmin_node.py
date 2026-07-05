# Copyright (C) 2023 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import atexit
import uuid
from threading import Thread, RLock

import yasmin_ros

import rclpy
from rclpy.node import Node

# Check if EventsExecutor is available
try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class YasminNode(Node):
    """
    A ROS 2 node for managing and handling YASMIN-based applications.

    YasminNode is a singleton class derived from Node and integrates
    custom functionalities for executing specific tasks in a ROS 2 environment.
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

    @staticmethod
    def destroy_instance() -> None:
        """
        Destroy the singleton instance if it exists.
        """
        with YasminNode._lock:
            if YasminNode._instance is not None:
                if yasmin_ros.logger_node is YasminNode._instance:
                    yasmin_ros.logger_node = None
                YasminNode._instance.shutdown()
                YasminNode._instance = None

    def __init__(self) -> None:
        """
        Default constructor. Initializes the node with a unique name.

        This constructor initializes the ROS 2 Node and
        starts an Executor for handling node callbacks.
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
        self._executor = Executor()
        self._executor.add_node(self)

        ## Thread to execute the spinning of the node.
        self._spin_thread: Thread = Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def shutdown(self) -> None:
        """
        Stop the executor thread and destroy the node.
        """
        if self._executor is not None:
            self._executor.remove_node(self)
            self._executor.shutdown()
            self._executor = None

        if self._spin_thread is not None:
            self._spin_thread.join()
            self._spin_thread = None

        self.destroy_node()


atexit.register(YasminNode.destroy_instance)
