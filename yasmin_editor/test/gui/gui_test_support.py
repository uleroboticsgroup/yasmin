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

from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field
from typing import List


@dataclass
class FakePluginInfo:
    plugin_type: str = "python"
    module: str = "demo_pkg.demo_state"
    class_name: str = "DemoState"
    outcomes: List[str] = field(default_factory=lambda: ["done", "failed"])
    package_name: str = "demo_pkg"
    file_name: str = "demo_state.xml"


class FakePluginManager:
    """Minimal plugin manager used by the GUI tests."""

    def __init__(self) -> None:
        self.python_plugins = [
            FakePluginInfo(module="demo_pkg.alpha", class_name="AlphaState"),
            FakePluginInfo(module="demo_pkg.beta", class_name="BetaState"),
        ]
        self.cpp_plugins = []
        self.xml_files = []

    def load_all_plugins(self) -> None:
        return None


def install_external_dependency_stubs(monkeypatch) -> None:
    """Install stub modules for optional runtime dependencies."""

    log_level = types.SimpleNamespace(
        ERROR="ERROR",
        WARN="WARN",
        INFO="INFO",
        DEBUG="DEBUG",
    )

    class Blackboard:
        def __init__(self, other=None) -> None:
            if other is None:
                self._data = {}
                self._remappings = {}
            else:
                self._data = other._data
                self._remappings = dict(other._remappings)

        def get(self, key):
            return self._data[key]

        def set(self, key, value):
            self._data[key] = value

        def remove(self, key):
            self._data.pop(key, None)

        def contains(self, key):
            return key in self._data

        def size(self):
            return len(self._data)

        def get_remappings(self):
            return dict(self._remappings)

        def set_remappings(self, remappings):
            self._remappings = dict(remappings)

        def keys(self):
            return self._data.keys()

        def items(self):
            return self._data.items()

        def values(self):
            return self._data.values()

        def __iter__(self):
            return iter(self._data)

    class RuntimeStateMachine:
        pass

    yasmin_module = types.ModuleType("yasmin")
    yasmin_module.LogLevel = log_level
    yasmin_module.Blackboard = Blackboard
    yasmin_module.StateMachine = RuntimeStateMachine
    yasmin_module.set_loggers = lambda *_args, **_kwargs: None
    yasmin_module.set_log_level = lambda *_args, **_kwargs: None
    yasmin_module.log_level_to_name = lambda level: str(level)

    yasmin_factory_module = types.ModuleType("yasmin_factory")

    class YasminFactory:
        def create_sm_from_file(self, _path: str):
            return RuntimeStateMachine()

    yasmin_factory_module.YasminFactory = YasminFactory

    plugin_manager_module = types.ModuleType("yasmin_plugins_manager.plugin_manager")
    plugin_manager_module.PluginManager = FakePluginManager
    plugin_manager_module.PluginInfo = FakePluginInfo

    plugin_info_module = types.ModuleType("yasmin_plugins_manager.plugin_info")
    plugin_info_module.PluginInfo = FakePluginInfo

    plugins_root_module = types.ModuleType("yasmin_plugins_manager")

    monkeypatch.setitem(sys.modules, "yasmin", yasmin_module)
    monkeypatch.setitem(sys.modules, "yasmin_factory", yasmin_factory_module)
    monkeypatch.setitem(sys.modules, "yasmin_plugins_manager", plugins_root_module)
    monkeypatch.setitem(
        sys.modules,
        "yasmin_plugins_manager.plugin_manager",
        plugin_manager_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "yasmin_plugins_manager.plugin_info",
        plugin_info_module,
    )
