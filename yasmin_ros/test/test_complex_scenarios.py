# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from yasmin import set_py_loggers, Blackboard, CbState, StateMachine
from yasmin import Concurrence, OrthogonalState, JoinState
from yasmin_ros import ActionState, ServiceState, MonitorState, PublisherState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT
from yasmin_ros.ros_clients_cache import ROSClientsCache
from yasmin_ros.yasmin_node import YasminNode

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String


class AuxNode(Node):
    """Auxiliary node providing action server, service server, and publisher."""

    def __init__(self):
        super().__init__("test_complex_node")

        self.action_server = ActionServer(
            self,
            Fibonacci,
            "test_action",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_action,
            cancel_callback=self.cancel_action,
            callback_group=ReentrantCallbackGroup(),
        )

        self.srv = self.create_service(
            AddTwoInts,
            "test_service",
            self.execute_service,
            callback_group=ReentrantCallbackGroup(),
        )

        self.pub = self.create_publisher(String, "test_topic", 10)
        self.monitor_timer = self.create_timer(0.5, self.publish_monitor_msg)
        self.monitor_msg_count = 0

    def goal_callback(self, goal_request) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        goal_handle.execute()

    def execute_action(self, goal_handle):
        result = Fibonacci.Result()
        request = goal_handle.request

        if request.order < 0:
            goal_handle.abort()
        else:
            if request.order == 0:
                time.sleep(2)
            else:
                time.sleep(1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.succeed()

        return result

    def cancel_action(self, goal_handle) -> int:
        return CancelResponse.ACCEPT

    def execute_service(self, request, response):
        response.sum = request.a + request.b
        return response

    def publish_monitor_msg(self):
        msg = String()
        msg.data = f"monitor_data_{self.monitor_msg_count}"
        self.monitor_msg_count += 1
        self.pub.publish(msg)


class TestComplexRosScenarios(unittest.TestCase):

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

    def setUp(self):
        self.bb = Blackboard()

    # ------------------------------------------------------------------ #
    # StateMachine chaining ROS states with CbState                      #
    # ------------------------------------------------------------------ #
    def test_sm_chain_action_cb_service(self):
        """StateMachine: ActionState -> CbState -> ServiceState.
        The action writes Fibonacci result to blackboard, CbState transforms it,
        ServiceState sends it."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 3
            return goal

        def action_result_handler(blackboard, result):
            blackboard["action_sum"] = 42
            return SUCCEED

        def process_cb(blackboard):
            val = blackboard.get("action_sum")
            blackboard["service_input_a"] = val
            blackboard["service_input_b"] = val * 2
            return "proceed"

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = blackboard.get("service_input_a")
            req.b = blackboard.get("service_input_b")
            return req

        def service_response_handler(blackboard, response):
            blackboard["final_sum"] = response.sum
            return SUCCEED

        action_outcomes = [SUCCEED, ABORT, CANCEL, TIMEOUT]
        service_outcomes = [SUCCEED, ABORT, CANCEL, TIMEOUT]

        sm = StateMachine([SUCCEED, "aborted", "canceled", "timeout"])
        sm.add_state(
            "FETCH_ACTION",
            ActionState(
                Fibonacci,
                "test_action",
                create_goal_cb,
                outcomes=action_outcomes,
                result_handler=action_result_handler,
            ),
            transitions={
                SUCCEED: "PROCESS",
                ABORT: "aborted",
                CANCEL: "canceled",
                TIMEOUT: "timeout",
            },
        )
        sm.add_state(
            "PROCESS",
            CbState(["proceed"], process_cb),
            transitions={"proceed": "SEND_SERVICE"},
        )
        sm.add_state(
            "SEND_SERVICE",
            ServiceState(
                AddTwoInts,
                "test_service",
                create_request_cb,
                outcomes=service_outcomes,
                response_handler=service_response_handler,
            ),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: "aborted",
                CANCEL: "canceled",
                TIMEOUT: "timeout",
            },
        )

        outcome = sm(self.bb)
        self.assertEqual(outcome, SUCCEED)
        self.assertIn("final_sum", self.bb)
        self.assertEqual(self.bb["final_sum"], 126)

    def test_sm_chain_action_publisher(self):
        """StateMachine: ActionState -> PublisherState.
        Action completes and then publishes a message."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def result_handler(blackboard, result):
            blackboard["publish_data"] = "action_done"
            return SUCCEED

        def create_msg_handler(blackboard):
            msg = String()
            msg.data = blackboard.get("publish_data")
            return msg

        action_outcomes = [SUCCEED, ABORT, CANCEL, TIMEOUT]

        sm = StateMachine([SUCCEED, ABORT, CANCEL, TIMEOUT])
        sm.add_state(
            "DO_ACTION",
            ActionState(
                Fibonacci,
                "test_action",
                create_goal_cb,
                outcomes=action_outcomes,
                result_handler=result_handler,
            ),
            transitions={
                SUCCEED: "PUBLISH",
                ABORT: ABORT,
                CANCEL: CANCEL,
                TIMEOUT: TIMEOUT,
            },
        )
        sm.add_state(
            "PUBLISH",
            PublisherState(String, "pub_topic", create_msg_handler),
            transitions={SUCCEED: SUCCEED},
        )

        outcome = sm(self.bb)
        self.assertEqual(outcome, SUCCEED)
        self.assertEqual(self.bb["publish_data"], "action_done")

    # ------------------------------------------------------------------ #
    # Concurrence with ROS states                                        #
    # ------------------------------------------------------------------ #
    def test_concurrence_action_and_monitor(self):
        """Concurrence running ActionState + MonitorState in parallel."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 3
            return goal

        def monitor_handler(blackboard, msg):
            blackboard["monitor_received"] = msg.data
            return SUCCEED

        action = ActionState(Fibonacci, "test_action", create_goal_cb)
        monitor = MonitorState(
            String,
            "test_topic",
            [SUCCEED],
            monitor_handler=monitor_handler,
            timeout=5,
        )

        conc = Concurrence(
            states={"ACTION": action, "MONITOR": monitor},
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"ACTION": SUCCEED, "MONITOR": SUCCEED}},
        )

        outcome = conc(self.bb)
        self.assertEqual(outcome, SUCCEED)

    def test_concurrence_action_and_service(self):
        """Concurrence running ActionState + ServiceState in parallel."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 10
            req.b = 20
            return req

        action = ActionState(Fibonacci, "test_action", create_goal_cb)
        service = ServiceState(AddTwoInts, "test_service", create_request_cb)

        conc = Concurrence(
            states={"ACTION": action, "SERVICE": service},
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"ACTION": SUCCEED, "SERVICE": SUCCEED}},
        )

        outcome = conc(self.bb)
        self.assertEqual(outcome, SUCCEED)

    def test_concurrence_with_cb_and_action(self):
        """Concurrence with ActionState and CbState running in parallel."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def cb_state_execute(blackboard):
            blackboard["cb_ran"] = True
            return "cb_done"

        action = ActionState(Fibonacci, "test_action", create_goal_cb)
        cb_state = CbState(["cb_done"], cb_state_execute)

        conc = Concurrence(
            states={"ACTION": action, "CB": cb_state},
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"ACTION": SUCCEED, "CB": "cb_done"}},
        )

        outcome = conc(self.bb)
        self.assertEqual(outcome, SUCCEED)

    def test_concurrence_monitor_and_service(self):
        """Concurrence running MonitorState + ServiceState in parallel."""

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 5
            req.b = 7
            return req

        def monitor_handler(blackboard, msg):
            return SUCCEED

        monitor = MonitorState(
            String,
            "test_topic",
            [SUCCEED],
            monitor_handler=monitor_handler,
            timeout=5,
        )
        service = ServiceState(AddTwoInts, "test_service", create_request_cb)

        conc = Concurrence(
            states={"MONITOR": monitor, "SERVICE": service},
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"MONITOR": SUCCEED, "SERVICE": SUCCEED}},
        )

        outcome = conc(self.bb)
        self.assertEqual(outcome, SUCCEED)

    def test_concurrence_mixed_outcome_mapping(self):
        """Concurrence where one child's outcome is mapped to a custom name."""

        def monitor_handler_special(blackboard, msg):
            return "special_outcome"

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 3
            req.b = 4
            return req

        monitor = MonitorState(
            String,
            "test_topic",
            ["special_outcome"],
            monitor_handler=monitor_handler_special,
            timeout=5,
        )
        service = ServiceState(AddTwoInts, "test_service", create_request_cb)

        conc = Concurrence(
            states={"MONITOR": monitor, "SERVICE": service},
            default_outcome=TIMEOUT,
            outcome_map={
                "custom_done": {"MONITOR": "special_outcome", "SERVICE": SUCCEED}
            },
        )

        outcome = conc(self.bb)
        self.assertEqual(outcome, "custom_done")

    # ------------------------------------------------------------------ #
    # OrthogonalState with ROS states                                    #
    # ------------------------------------------------------------------ #
    def test_orthogonal_with_ros_regions(self):
        """OrthogonalState with two regions:
        Region A: StateMachine with ActionState
        Region B: StateMachine with ServiceState
        """

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 7
            req.b = 8
            return req

        ros_outcomes = [SUCCEED, ABORT, CANCEL, TIMEOUT]

        region_a = StateMachine(ros_outcomes)
        region_a.set_name("RegionA")
        region_a.add_state(
            "ACTION",
            ActionState(Fibonacci, "test_action", create_goal_cb, outcomes=ros_outcomes),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT,
                CANCEL: CANCEL,
                TIMEOUT: TIMEOUT,
            },
        )

        region_b = StateMachine(ros_outcomes)
        region_b.set_name("RegionB")
        region_b.add_state(
            "SERVICE",
            ServiceState(
                AddTwoInts, "test_service", create_request_cb, outcomes=ros_outcomes
            ),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT,
                CANCEL: CANCEL,
                TIMEOUT: TIMEOUT,
            },
        )

        ort = OrthogonalState(
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"RegionA": SUCCEED, "RegionB": SUCCEED}},
        )
        ort.add_region("RegionA", region_a)
        ort.add_region("RegionB", region_b)
        ort.configure()

        outcome = ort(self.bb)
        self.assertEqual(outcome, SUCCEED)

    def test_orthogonal_with_join_and_ros(self):
        """OrthogonalState with JoinState sync between two regions,
        each region containing ROS states."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 10
            req.b = 20
            return req

        ros_outcomes = [SUCCEED]

        region_a = StateMachine(ros_outcomes)
        region_a.set_name("RegionA")
        region_a.add_state(
            "ACTION",
            ActionState(Fibonacci, "test_action", create_goal_cb, outcomes=ros_outcomes),
            transitions={SUCCEED: "JOIN"},
        )
        region_a.add_state(
            "JOIN", JoinState("sync_point"), transitions={"joined": "DONE"}
        )
        region_a.add_state(
            "DONE", CbState([SUCCEED], lambda bb: SUCCEED), transitions={SUCCEED: SUCCEED}
        )

        region_b = StateMachine(ros_outcomes)
        region_b.set_name("RegionB")
        region_b.add_state(
            "SERVICE",
            ServiceState(
                AddTwoInts, "test_service", create_request_cb, outcomes=ros_outcomes
            ),
            transitions={SUCCEED: "JOIN"},
        )
        region_b.add_state(
            "JOIN", JoinState("sync_point"), transitions={"joined": "DONE"}
        )
        region_b.add_state(
            "DONE", CbState([SUCCEED], lambda bb: SUCCEED), transitions={SUCCEED: SUCCEED}
        )

        ort = OrthogonalState(
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"RegionA": SUCCEED, "RegionB": SUCCEED}},
        )
        ort.add_region("RegionA", region_a)
        ort.add_region("RegionB", region_b)
        ort.configure()

        outcome = ort(self.bb)
        self.assertEqual(outcome, SUCCEED)

    # ------------------------------------------------------------------ #
    # StateMachine with nested Concurrence of ROS states                 #
    # ------------------------------------------------------------------ #
    def test_sm_with_nested_concurrence_of_ros_states(self):
        """StateMachine where one state is a Concurrence of ROS states."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 3
            req.b = 4
            return req

        sm = StateMachine([SUCCEED, TIMEOUT])

        sm.add_state(
            "START",
            CbState(["go_next"], lambda bb: "go_next"),
            transitions={"go_next": "PARALLEL"},
        )

        action = ActionState(Fibonacci, "test_action", create_goal_cb)
        service = ServiceState(AddTwoInts, "test_service", create_request_cb)

        conc = Concurrence(
            states={"ACTION": action, "SERVICE": service},
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"ACTION": SUCCEED, "SERVICE": SUCCEED}},
        )

        sm.add_state(
            "PARALLEL",
            conc,
            transitions={SUCCEED: SUCCEED, TIMEOUT: TIMEOUT},
        )

        outcome = sm(self.bb)
        self.assertEqual(outcome, SUCCEED)

    # ------------------------------------------------------------------ #
    # StateMachine with nested OrthogonalState of ROS regions            #
    # ------------------------------------------------------------------ #
    def test_sm_with_orthogonal_of_ros_regions(self):
        """StateMachine where one state is an OrthogonalState with ROS regions."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def create_request_cb(blackboard):
            req = AddTwoInts.Request()
            req.a = 5
            req.b = 6
            return req

        ros_outcomes = [SUCCEED]

        region_a = StateMachine(ros_outcomes)
        region_a.set_name("RegionA")
        region_a.add_state(
            "ACTION",
            ActionState(Fibonacci, "test_action", create_goal_cb, outcomes=ros_outcomes),
            transitions={SUCCEED: SUCCEED},
        )

        region_b = StateMachine(ros_outcomes)
        region_b.set_name("RegionB")
        region_b.add_state(
            "SERVICE",
            ServiceState(
                AddTwoInts, "test_service", create_request_cb, outcomes=ros_outcomes
            ),
            transitions={SUCCEED: SUCCEED},
        )

        ort = OrthogonalState(
            default_outcome=TIMEOUT,
            outcome_map={SUCCEED: {"RegionA": SUCCEED, "RegionB": SUCCEED}},
        )
        ort.add_region("RegionA", region_a)
        ort.add_region("RegionB", region_b)
        ort.configure()

        sm = StateMachine([SUCCEED, TIMEOUT])
        sm.add_state(
            "START",
            CbState(["go"], lambda bb: "go"),
            transitions={"go": "ORTHO"},
        )
        sm.add_state(
            "ORTHO",
            ort,
            transitions={SUCCEED: SUCCEED, TIMEOUT: TIMEOUT},
        )

        outcome = sm(self.bb)
        self.assertEqual(outcome, SUCCEED)

    # ------------------------------------------------------------------ #
    # Blackboard data flow through sequential StateMachine                #
    # ------------------------------------------------------------------ #
    def test_blackboard_flow_through_state_machine(self):
        """StateMachine: ActionState -> CbState where CbState reads action's
        blackboard output."""

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def result_handler(blackboard, result):
            blackboard["action_result"] = "from_action"
            return SUCCEED

        def check_bb_cb(blackboard):
            if blackboard.contains("action_result"):
                blackboard["check_passed"] = True
            else:
                blackboard["check_passed"] = False
            return "checked"

        ros_outcomes = [SUCCEED, ABORT, CANCEL, TIMEOUT]

        sm = StateMachine([SUCCEED, "checked"])
        sm.add_state(
            "ACTION",
            ActionState(
                Fibonacci,
                "test_action",
                create_goal_cb,
                outcomes=ros_outcomes,
                result_handler=result_handler,
            ),
            transitions={
                SUCCEED: "CHECK",
                ABORT: SUCCEED,
                CANCEL: SUCCEED,
                TIMEOUT: SUCCEED,
            },
        )
        sm.add_state(
            "CHECK",
            CbState(["checked"], check_bb_cb),
            transitions={"checked": SUCCEED},
        )

        outcome = sm(self.bb)
        self.assertEqual(outcome, SUCCEED)
        self.assertTrue(self.bb["check_passed"])
        self.assertEqual(self.bb["action_result"], "from_action")


if __name__ == "__main__":
    unittest.main()
