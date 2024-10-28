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


import uuid
from threading import Thread, RLock

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import yasmin


class YasminNode(Node):

    _instance: "YasminNode" = None
    _lock: RLock = RLock()
    _executor: MultiThreadedExecutor = None
    _spin_thread: Thread = None

    @staticmethod
    def get_instance() -> "YasminNode":

        with YasminNode._lock:
            if YasminNode._instance == None:
                YasminNode._instance = YasminNode()

            return YasminNode._instance

    def __init__(self) -> None:

        if not YasminNode._instance is None:
            raise Exception("This class is a Singleton")

        super().__init__(f"yasmin_{str(uuid.uuid4()).replace('-', '_')}_node")

        yasmin.set_loggers(
            self.get_logger().info,
            self.get_logger().warn,
            self.get_logger().debug,
            self.get_logger().error,
        )

        # executor
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()
