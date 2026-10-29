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

import pytest
import rclpy

from yasmin_ros.yasmin_node import YasminNode


@pytest.fixture(autouse=True)
def ros_context(monkeypatch):
    monkeypatch.delenv("YASMIN_USE_SIM_TIME", raising=False)
    rclpy.init(args=[])
    try:
        yield
    finally:
        YasminNode.destroy_instance()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "value, expected",
    [
        (None, False),
        ("true", True),
        ("True", True),
        ("TRUE", True),
        ("1", True),
        ("false", False),
        ("False", False),
        ("0", False),
        (" \tTrue\n", True),
    ],
)
def test_sim_time_environment(monkeypatch, value, expected):
    if value is not None:
        monkeypatch.setenv("YASMIN_USE_SIM_TIME", value)

    node = YasminNode.get_instance()
    assert node.get_parameter("use_sim_time").value is expected
    assert node.get_clock().ros_time_is_active is expected


@pytest.mark.parametrize("value", ["", "yes", "2"])
def test_invalid_sim_time_environment(monkeypatch, value):
    monkeypatch.setenv("YASMIN_USE_SIM_TIME", value)
    with pytest.raises(ValueError, match="YASMIN_USE_SIM_TIME"):
        YasminNode.get_instance()
    assert YasminNode._instance is None
