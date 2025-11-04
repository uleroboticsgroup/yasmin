# Copyright (C) 2023 Miguel Ángel González Santamarta

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


import unittest
from threading import Thread

from yasmin import set_py_loggers
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class AuxNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.pub = self.create_publisher(String, "test", 10)
        self.timer = self.create_timer(1, self.publish_msgs)

    def publish_msgs(self) -> None:
        msg = String()
        msg.data = "data"
        self.pub.publish(msg)


class TestYasminRos(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_py_loggers()

        cls.aux_node = AuxNode()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.aux_node)

        cls.spin_thread = Thread(target=cls.executor.spin)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_monitor_timeout(self):

        def monitor_handler(blackboard, msg):
            return SUCCEED

        state = MonitorState(
            String,
            "test1",
            [SUCCEED],
            monitor_handler=monitor_handler,
            timeout=2,
        )
        self.assertEqual(TIMEOUT, state())

    def test_monitor_retry_timeout(self):
        def monitor_handler(blackboard, msg):
            return SUCCEED

        retries = 3

        ## Capture the logs
        with self.assertLogs("root", level="WARNING") as captured:
            state = MonitorState(
                String,
                "test1",
                [SUCCEED],
                monitor_handler=monitor_handler,
                timeout=0.1,
                maximum_retry=retries,
            )
            self.assertEqual(TIMEOUT, state())

        ## Check that the number of WARNING logs is correct
        self.assertEqual(
            (retries * 2) + 1,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )


if __name__ == "__main__":
    unittest.main()
