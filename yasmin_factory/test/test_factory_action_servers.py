# Copyright (C) 2026 Maik Knof
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

import os
import signal
import subprocess
import tempfile
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from yasmin_msgs.action import RunStateMachine

ROS_LOG_DIR = os.path.join(tempfile.gettempdir(), "yasmin_factory_action_test_logs")
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ["ROS_LOG_DIR"] = ROS_LOG_DIR
os.environ["ROS_DOMAIN_ID"] = str(200 + os.getpid() % 30)


SERVER_EXECUTABLES = [
    "yasmin_factory_action_server",
    "yasmin_factory_action_server.py",
]


def spin_until_complete(node, future, timeout=10.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), "ROS action operation timed out"
    return future.result()


@pytest.fixture(params=SERVER_EXECUTABLES)
def action_server(request):
    action_name = f"run_state_machine_{uuid.uuid4().hex}"
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "yasmin_factory",
            request.param,
            "--ros-args",
            "-p",
            "enable_viewer_pub:=false",
            "-r",
            f"run_state_machine:={action_name}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    node = rclpy.create_node(f"test_{request.param.replace('.', '_')}")
    client = ActionClient(node, RunStateMachine, action_name)
    assert client.wait_for_server(timeout_sec=10.0), f"{request.param} did not start"

    yield node, client

    client.destroy()
    node.destroy_node()
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)
        pytest.fail(f"{request.param} did not shut down cleanly")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def fixture_path(filename):
    return os.path.join(get_package_share_directory("yasmin_factory"), "test", filename)


def test_goal_completes(action_server):
    node, client = action_server
    goal = RunStateMachine.Goal()
    goal.state_machine_file = fixture_path("test_action_complete.xml")
    feedback_states = []

    goal_handle = spin_until_complete(
        node,
        client.send_goal_async(
            goal,
            feedback_callback=lambda msg: feedback_states.append(
                msg.feedback.current_state
            ),
        ),
    )
    assert goal_handle.accepted
    result = spin_until_complete(node, goal_handle.get_result_async())

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.outcome == "done"
    assert feedback_states[0] == "CompleteState"


def test_goal_is_canceled(action_server):
    node, client = action_server
    goal = RunStateMachine.Goal()
    goal.state_machine_file = fixture_path("test_action_cancel.xml")
    feedback_states = []

    goal_handle = spin_until_complete(
        node,
        client.send_goal_async(
            goal,
            feedback_callback=lambda msg: feedback_states.append(
                msg.feedback.current_state
            ),
        ),
    )
    assert goal_handle.accepted

    deadline = time.monotonic() + 5.0
    while not feedback_states and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert feedback_states == ["SlowState"]

    cancel_response = spin_until_complete(node, goal_handle.cancel_goal_async())
    assert cancel_response.goals_canceling
    result = spin_until_complete(node, goal_handle.get_result_async())

    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.outcome == ""
