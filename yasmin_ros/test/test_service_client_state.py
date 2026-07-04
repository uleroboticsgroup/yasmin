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


import time
import unittest
from threading import Thread

from yasmin import set_py_loggers
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


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
        ROSClientsCache.clear_all()
        YasminNode.destroy_instance()
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
        with self.assertLogs(level="WARNING") as captured:
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
            retries,
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
        with self.assertLogs(level="WARNING") as captured:
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
            retries,
            len(captured.records),
            msg=f"Expected {retries} WARNING logs, saw {len(captured)}.\n"
            f"Captured messages: {[r.getMessage() for r in captured.records]}",
        )


if __name__ == "__main__":
    unittest.main()
