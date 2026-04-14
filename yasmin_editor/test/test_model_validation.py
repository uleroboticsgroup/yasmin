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

"""Validation tests for state-machine and concurrence model constraints."""

from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.transition import Transition
from yasmin_editor.model.validation import validate_model


def make_py_state(name: str, outcomes=None, **kwargs) -> State:
    return State(
        name=name,
        state_type="py",
        module="demo.module",
        class_name="DemoState",
        outcomes=[Outcome(item) for item in (outcomes or ["done"])],
        **kwargs,
    )


def test_validate_model_accepts_well_formed_state_machine():
    root = StateMachine(name="root", outcomes=[Outcome("done")], start_state="worker")
    worker = make_py_state("worker", ["success"])
    root.add_state(worker)
    root.add_transition("worker", Transition("success", "done"))

    result = validate_model(root)

    assert result.is_valid
    assert result.errors == []
    assert result.warnings == []


def test_validate_model_reports_invalid_state_machine_and_child_issues():
    root = StateMachine(name="root", outcomes=[Outcome("done")], start_state="missing")
    invalid_child = State(
        name="worker",
        state_type="py",
        class_name=None,
        module=None,
        outcomes=[Outcome("go"), Outcome("go")],
        keys=[Key("input"), Key("input")],
        remappings={"": "target", "source": ""},
    )
    root.states["worker"] = invalid_child
    root.transitions["worker"] = [Transition("go", "ghost")]
    root.transitions["ghost_owner"] = [Transition("x", "done")]

    result = validate_model(root)
    messages = {f"{item.path}: {item.message}" for item in result.errors}

    assert result.is_valid is False
    assert "root: Start state 'missing' does not exist" in messages
    assert "root/worker: Python state requires 'module'" in messages
    assert "root/worker: Python state requires 'class_name'" in messages
    assert "root/worker: Duplicate outcome 'go'" in messages
    assert "root/worker: Duplicate key 'input'" in messages
    assert "root/worker: Transition target 'ghost' does not exist" in messages
    assert "root/worker: Remapping source key must not be empty" in messages
    assert "root/worker: Remapping target key must not be empty" in messages
    assert "root: Transitions defined for unknown owner 'ghost_owner'" in messages


def test_validate_model_reports_container_name_conflicts_and_container_transitions():
    root = StateMachine(name="root", outcomes=[Outcome("done"), Outcome("worker")])
    root.states["worker"] = make_py_state("worker", ["go"])
    root.transitions[root.name] = [Transition("done", "missing_parent_target")]

    result = validate_model(root)
    messages = {f"{item.path}: {item.message}" for item in result.errors}

    assert (
        "root: Name 'worker' is used by both a child state and a final outcome"
        in messages
    )
    assert (
        "root: Container transition target 'missing_parent_target' does not exist"
        in messages
    )


def test_validate_model_reports_concurrence_specific_problems():
    root = StateMachine(name="root", outcomes=[Outcome("done")], start_state="cc")
    cc = Concurrence(
        name="cc",
        outcomes=[Outcome("done")],
        default_outcome="missing",
    )
    cc.states["worker"] = make_py_state("worker", ["ok"])
    cc.outcome_map["missing_outcome"] = {"worker": "bad_child_outcome", "ghost": "ok"}
    root.add_state(cc)
    root.add_transition("cc", Transition("done", "done"))

    result = validate_model(root)

    error_messages = {f"{item.path}: {item.message}" for item in result.errors}
    warning_messages = {f"{item.path}: {item.message}" for item in result.warnings}

    assert "root/cc: Default outcome 'missing' does not exist" in error_messages
    assert (
        "root/cc: Outcome map references unknown outcome 'missing_outcome'"
        in error_messages
    )
    assert "root/cc: Outcome map references unknown state 'ghost'" in error_messages
    assert (
        "root/cc/worker: Outcome map references unknown outcome 'bad_child_outcome'"
        in warning_messages
    )
