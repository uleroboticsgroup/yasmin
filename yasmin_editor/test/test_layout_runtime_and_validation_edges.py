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

import importlib
import json
import sys
import types
from pathlib import Path

import pytest

from yasmin_editor.editor_gui.recent_files import RecentFilesStore, prune_recent_file_entries
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.layout import Layout
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import ValidationResult, validate_model


def make_leaf(name: str, outcomes=None, **kwargs) -> State:
    return State(
        name=name,
        outcomes=[Outcome(item) for item in (outcomes or ["done"])],
        **kwargs,
    )


def test_layout_tracks_aliases_primary_positions_and_removals():
    layout = Layout()

    layout.set_state_position("worker", 1.0, 2.0)
    assert layout.get_state_position("worker").x == 1.0

    layout.rename_state_position("worker", "helper")
    assert layout.get_state_position("worker") is None
    assert layout.get_state_position("helper").y == 2.0

    layout.set_primary_outcome_position("done", 3.0, 4.0)
    first_id = layout.materialize_primary_outcome_position("done")
    second_id = layout.create_outcome_alias("done", 5.0, 6.0)

    assert first_id is not None
    assert second_id != first_id
    assert len(layout.get_outcome_placements("done")) == 2
    assert layout.get_outcome_placement(first_id).position.x == 3.0

    layout.set_outcome_position("done", 7.0, 8.0)
    assert layout.get_outcome_position("done").x == 7.0

    layout.remove_outcome_placement(second_id)
    assert len(layout.get_outcome_placements("done")) == 1
    assert layout.get_outcome_position("done").y == 8.0

    layout.rename_outcome_position("done", "finished")
    assert layout.get_outcome_position("done") is None
    assert layout.get_outcome_position("finished").x == 7.0

    layout.remove_outcome_position("finished")
    layout.remove_state_position("helper")
    assert layout.get_outcome_position("finished") is None
    assert layout.get_state_position("helper") is None


def test_recent_files_store_handles_invalid_payloads_and_existing_only_filter(tmp_path: Path):
    existing = tmp_path / "existing.xml"
    missing = tmp_path / "missing.xml"
    existing.write_text("<xml />", encoding="utf-8")

    assert prune_recent_file_entries(
        [str(existing), str(missing)],
        existing_only=True,
    ) == [str(existing.resolve())]

    store_path = tmp_path / "recent.json"
    store = RecentFilesStore(store_path, max_entries=5)

    store_path.write_text("{bad json", encoding="utf-8")
    assert store.load_entries() == []

    store_path.write_text(json.dumps({"unexpected": []}), encoding="utf-8")
    assert store.load_entries() == []

    store.clear()
    store.clear()


def test_runtime_package_keeps_runtime_export_lazy(monkeypatch):
    runtime_pkg = importlib.import_module("yasmin_editor.runtime")
    fake_runtime_module = types.ModuleType("yasmin_editor.runtime.runtime")

    class FakeRuntime:
        pass

    fake_runtime_module.Runtime = FakeRuntime
    monkeypatch.setitem(sys.modules, "yasmin_editor.runtime.runtime", fake_runtime_module)

    assert runtime_pkg.Runtime is FakeRuntime

    with pytest.raises(AttributeError):
        getattr(runtime_pkg, "MissingRuntime")


def test_validation_result_extend_and_string_representation_accumulate_messages():
    first = ValidationResult()
    first.add_error("root", "broken")
    second = ValidationResult()
    second.add_warning("root/state", "careful")

    first.extend(second)

    assert first.is_valid is False
    rendered = str(first)
    assert "Errors:" in rendered
    assert "Warnings:" in rendered
    assert "root: broken" in rendered
    assert "root/state: careful" in rendered


def test_validate_model_reports_additional_leaf_and_container_edge_cases():
    root = StateMachine(name="root")
    root.states["alias"] = make_leaf(
        "real_name",
        ["done", "done"],
        state_type="cpp",
        class_name=None,
        keys=[Key("input"), Key("input")],
        remappings={"": "target", "source": ""},
    )
    root.states["xml_state"] = make_leaf(
        "xml_state",
        state_type="xml",
        file_name=None,
        package_name=None,
    )
    root.states["leaf_without_type"] = make_leaf("leaf_without_type", state_type=None)
    root.transitions[root.name] = [Transition("unknown_outcome", "ghost")]

    result = validate_model(root)
    error_messages = {f"{item.path}: {item.message}" for item in result.errors}
    warning_messages = {f"{item.path}: {item.message}" for item in result.warnings}

    assert "root: State machine requires at least one outcome" in error_messages
    assert "root: State machine requires 'start_state'" in error_messages
    assert (
        "root/alias: Dictionary key 'alias' does not match state name 'real_name'"
        in error_messages
    )
    assert "root/alias: C++ state requires 'class_name'" in error_messages
    assert "root/alias: Duplicate outcome 'done'" in error_messages
    assert "root/alias: Duplicate key 'input'" in error_messages
    assert "root/alias: Remapping source key must not be empty" in error_messages
    assert "root/alias: Remapping target key must not be empty" in error_messages
    assert "root/xml_state: XML state requires 'file_name'" in error_messages
    assert "root: Container transition target 'ghost' does not exist" in error_messages
    assert (
        "root: Container transition uses unknown outcome 'unknown_outcome'"
        in warning_messages
    )
    assert "root/xml_state: XML state usually should define 'package_name'" in warning_messages
    assert "root/leaf_without_type: Leaf state has no 'state_type'" in warning_messages


def test_validate_model_reports_invalid_final_outcome_transition_targets():
    root = StateMachine(name="root", outcomes=[Outcome("done")], start_state="worker")
    root.add_state(make_leaf("worker", ["ok"], state_type="py", module="demo.module", class_name="DemoState"))
    root.transitions["done"] = [Transition("again", "ghost")]

    result = validate_model(root)
    error_messages = {f"{item.path}: {item.message}" for item in result.errors}

    assert "root: Final outcome transition target 'ghost' does not exist" in error_messages


def test_validate_model_warns_for_empty_concurrence():
    result = validate_model(Concurrence(name="cc"))

    warning_messages = {f"{item.path}: {item.message}" for item in result.warnings}
    error_messages = {f"{item.path}: {item.message}" for item in result.errors}

    assert "cc: Concurrence has no child states" in warning_messages
    assert "cc: Concurrence requires at least one outcome" in error_messages
