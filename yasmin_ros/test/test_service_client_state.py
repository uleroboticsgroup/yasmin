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


import time
import unittest
from threading import Thread

from yasmin import set_py_loggers
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class AuxNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.service_server = self.create_service(
            AddTwoInts, "test", self.execute_service
        )

        self.pub = self.create_publisher(String, "test", 10)

    def execute_service(self, request, response):
        time.sleep(2)
        response.sum = request.a + request.b
        return response


class TestServiceClientState(unittest.TestCase):

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

    def test_service_client(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        state = ServiceState(AddTwoInts, "test", create_request_cb)
        self.assertEqual(SUCCEED, state())

    def test_service_client_cache(self):
        ROSClientsCache.clear_all()
        self.assertEqual(0, ROSClientsCache.get_service_clients_count())

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        state1 = ServiceState(AddTwoInts, "test", create_request_cb)
        self.assertEqual(1, ROSClientsCache.get_service_clients_count())

        state2 = ServiceState(AddTwoInts, "test", create_request_cb)
        self.assertEqual(1, ROSClientsCache.get_service_clients_count())

        state3 = ServiceState(AddTwoInts, "test2", create_request_cb)
        self.assertEqual(2, ROSClientsCache.get_service_clients_count())

    def test_service_client_response_handler(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        def response_handler(blackboard, response):
            return "new_outcome"

        state = ServiceState(
            AddTwoInts, "test", create_request_cb, ["new_outcome"], response_handler
        )
        self.assertEqual("new_outcome", state())

    def test_service_client_retry_wait_timeout(self):
        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        retries = 3

        ## Capture the logs
        with self.assertLogs("root", level="WARNING") as captured:
            state = ServiceState(
                AddTwoInts,
                "test_retry",
                create_request_cb,
                maximum_retry=retries,
                wait_timeout=0.1,
            )
            self.assertEqual(TIMEOUT, state())

        ## Check that the number of WARNING logs is correct
        self.assertEqual(
            (retries * 2) + 1,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )

    def test_service_client_retry_response_timeout(self):
        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        retries = 3

        ## Capture the logs
        with self.assertLogs("root", level="WARNING") as captured:
            state = ServiceState(
                AddTwoInts,
                "test",
                create_request_cb,
                maximum_retry=retries,
                response_timeout=0.1,
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
