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


import unittest
from threading import Thread

from yasmin import set_py_loggers
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.ros_clients_cache import ROSClientsCache

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


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
        ROSClientsCache.clear_all()
        YasminNode.destroy_instance()
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
        with self.assertLogs(level="WARNING") as captured:
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
            retries,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )


if __name__ == "__main__":
    unittest.main()
