# Copyright (C) 2026 Maik Knof
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Helpers shared by the GUI tests.
#
# These fakes keep the tests deterministic and avoid importing the full runtime
# stack when only editor widgets are under test.

from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field


@dataclass
class FakePluginInfo:
    plugin_type: str = "python"
    module: str = "demo_pkg.demo_state"
    class_name: str = "DemoState"
    outcomes: list[str] = field(default_factory=lambda: ["done", "failed"])
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
        def __init__(self) -> None:
            self._data = {}
            self._remappings = {}

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
