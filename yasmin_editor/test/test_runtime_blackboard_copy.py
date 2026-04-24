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

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import pytest

pytest.importorskip("PyQt5.QtCore")


def _install_runtime_test_stubs(monkeypatch):
    class Blackboard:
        def __init__(self, other=None) -> None:
            if other is None:
                self._data = {}
                self._remappings = {}
            else:
                self._data = other._data
                self._remappings = dict(other._remappings)

        def get(self, key):
            return self._data[self._remappings.get(key, key)]

        def set(self, key, value):
            self._data[self._remappings.get(key, key)] = value

        def remove(self, key):
            self._data.pop(self._remappings.get(key, key), None)

        def contains(self, key):
            return self._remappings.get(key, key) in self._data

        def size(self):
            return len(self._data)

        def get_remappings(self):
            return dict(self._remappings)

        def set_remappings(self, remappings):
            self._remappings = dict(remappings)

        def keys(self):
            return list(self._data.keys())

        def items(self):
            return list(self._data.items())

        def values(self):
            return list(self._data.values())

        def __iter__(self):
            return iter(self.keys())

    class RuntimeStateMachine:
        def add_start_cb(self, *_args, **_kwargs):
            return None

        def add_transition_cb(self, *_args, **_kwargs):
            return None

        def add_end_cb(self, *_args, **_kwargs):
            return None

        def cancel_state(self):
            return None

        def cancel_state_machine(self):
            return None

        def __call__(self, _blackboard):
            return "done"

    class FakeFactory:
        def create_sm_from_file(self, _path):
            return RuntimeStateMachine()

    yasmin_module = types.ModuleType("yasmin")
    yasmin_module.Blackboard = Blackboard
    yasmin_module.StateMachine = RuntimeStateMachine
    yasmin_module.LogLevel = types.SimpleNamespace(
        ERROR="ERROR",
        WARN="WARN",
        INFO="INFO",
        DEBUG="DEBUG",
    )
    yasmin_module.set_loggers = lambda *_args, **_kwargs: None
    yasmin_module.set_log_level = lambda *_args, **_kwargs: None
    yasmin_module.log_level_to_name = lambda level: str(level)

    factory_module = types.ModuleType("yasmin_factory")
    factory_module.YasminFactory = FakeFactory

    monkeypatch.setitem(sys.modules, "yasmin", yasmin_module)
    monkeypatch.setitem(sys.modules, "yasmin_factory", factory_module)

    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

    module_name = "yasmin_editor.runtime.runtime"
    sys.modules.pop(module_name, None)
    return importlib.import_module(module_name)


def test_runtime_uses_blackboard_copy_for_shell_and_keeps_transition_remappings(
    monkeypatch,
):
    runtime_module = _install_runtime_test_stubs(monkeypatch)
    Runtime = runtime_module.Runtime

    runtime = Runtime()
    runtime._register_callbacks = lambda: None

    assert runtime.create_sm_from_file("/tmp/runtime_test.xml") is True
    assert runtime.bb is not runtime.shell_bb

    runtime.bb.set("shared", 1)
    assert runtime.shell_bb.get("shared") == 1

    runtime.bb.set_remappings({"alias": "shared"})
    assert runtime.bb.get_remappings() == {"alias": "shared"}
    assert runtime.shell_bb.get_remappings() == {}
    assert not runtime.shell_bb.contains("alias")

    runtime.shell_bb.set_remappings({"shell_alias": "shared"})
    assert runtime.bb.get_remappings() == {"alias": "shared"}
    assert runtime.shell_bb.get_remappings() == {"shell_alias": "shared"}
    assert runtime.shell_bb.get("shell_alias") == 1

    runtime.bb.set("shared", 2)
    assert runtime.bb.get("shared") == 2
    assert runtime.shell_bb.get("shared") == 2

    runtime._expand_to_deepest_known_path = lambda path: tuple(path)
    runtime._resolve_state_reference = lambda path: tuple(path)
    runtime._update_shell_state_refs = lambda *_args, **_kwargs: None
    runtime._request_breakpoint_pause = lambda *_args, **_kwargs: None
    runtime._pause_if_requested = lambda: None
    runtime._has_breakpoint = lambda _path: False

    original_remappings = runtime.bb.get_remappings()
    runtime.transition_cb(runtime.bb, "worker", "done", "success")
    assert runtime.bb.get_remappings() == original_remappings
