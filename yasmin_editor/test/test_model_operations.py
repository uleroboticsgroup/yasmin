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

"""Round-trip and mutation tests for the editor model and XML conversion."""

import pytest

from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition


def make_leaf(name: str, outcomes=None, **kwargs) -> State:
    return State(
        name=name,
        state_type="py",
        module="demo.module",
        class_name="DemoState",
        outcomes=[Outcome(item) for item in (outcomes or ["done"])],
        **kwargs,
    )


def test_state_helpers_and_string_representation_cover_leaf_metadata():
    state = State(
        name="worker",
        description="Runs one task",
        state_type="py",
        module="demo.module",
        class_name="DemoState",
        package_name="demo_pkg",
        file_name="demo.py",
        remappings={"input": "shared_input"},
        parameter_mappings={"rate": "shared_rate"},
    )

    state.add_key(Key("input", default_type="str", default_value="value"))
    state.add_parameter(Parameter("rate", default_type="int", default_value=10))
    state.add_outcome(Outcome("done"))
    state.add_outcome(Outcome("failed"))

    assert state.is_container is False
    assert state.is_leaf is True
    assert state.get_outcome("done").name == "done"
    assert state.keys[0].has_default is True
    assert state.parameters[0].has_default is True

    rendered = str(state)
    assert "State(worker" in rendered
    assert "type=py" in rendered
    assert "class=DemoState" in rendered
    assert "module=demo.module" in rendered
    assert "package=demo_pkg" in rendered
    assert "file=demo.py" in rendered
    assert "outcomes: done, failed" in rendered
    assert "params: rate" in rendered
    assert "keys: input" in rendered
    assert "param remap: rate->shared_rate" in rendered
    assert "remap: input->shared_input" in rendered
    assert "description: Runs one task" in rendered


def test_state_rename_outcome_rejects_duplicates_and_ignores_missing_names():
    state = make_leaf("worker", ["done", "failed"])

    state.rename_outcome("done", "success")

    assert [outcome.name for outcome in state.outcomes] == ["success", "failed"]

    with pytest.raises(ValueError, match="already exists"):
        state.rename_outcome("success", "failed")

    state.rename_outcome("missing", "ignored")
    assert [outcome.name for outcome in state.outcomes] == ["success", "failed"]


def test_state_machine_operations_update_transitions_layout_and_text_blocks():
    root = StateMachine(
        name="root",
        description="Top level",
        outcomes=[Outcome("done"), Outcome("aborted")],
        start_state="alpha",
    )
    alpha = make_leaf("alpha", ["ok", "retry"], remappings={"input": "shared"})
    beta = make_leaf("beta", ["done"])
    note = TextBlock(x=10.0, y=20.0, content="hello\nworld")

    root.add_state(alpha)
    root.add_state(beta)
    root.add_text_block(note)
    root.layout.set_state_position("beta", 5.0, 6.0)
    root.layout.set_primary_outcome_position("done", 7.0, 8.0)
    root.layout.materialize_primary_outcome_position("done")
    root.add_transition("alpha", Transition("ok", "beta", target_instance_id="one"))
    root.add_transition("alpha", Transition("ok", "beta", target_instance_id="two"))
    root.add_transition("alpha", Transition("retry", "done"))
    root.add_transition("alpha", Transition("ok", "done"))
    root.transitions["orphan"] = [Transition("finished", "aborted")]

    assert len(root.transitions["alpha"]) == 3
    assert root.transitions["alpha"][0].target_instance_id == "two"

    root.rename_transition_owner("orphan", "alpha")
    assert ("finished", "aborted") in {
        (item.source_outcome, item.target) for item in root.transitions["alpha"]
    }

    root.rename_state("beta", "gamma")
    assert root.get_state("beta") is None
    assert root.get_state("gamma") is beta
    assert beta.name == "gamma"
    assert root.layout.get_state_position("gamma").x == 5.0
    assert any(item.target == "gamma" for item in root.transitions["alpha"])

    root.rename_child_state_outcome("alpha", "retry", "ok")
    alpha_edges = {
        (item.source_outcome, item.target) for item in root.transitions["alpha"]
    }
    assert alpha_edges == {
        ("ok", "gamma"),
        ("ok", "done"),
        ("finished", "aborted"),
    }

    root.rename_outcome("done", "finished")
    assert root.layout.get_outcome_position("finished").x == 7.0
    assert root.get_outcome("finished") is not None
    assert root.get_outcome("done") is None
    assert ("ok", "finished") in {
        (item.source_outcome, item.target) for item in root.transitions["alpha"]
    }

    root.remove_transition("alpha", "ok", "gamma")
    assert ("ok", "gamma") not in {
        (item.source_outcome, item.target) for item in root.transitions["alpha"]
    }

    root.remove_state("gamma")
    assert root.get_state("gamma") is None
    assert root.layout.get_state_position("gamma") is None

    root.remove_outcome("finished")
    assert root.get_outcome("finished") is None
    assert root.layout.get_outcome_position("finished") is None

    rendered = str(root)
    assert "StateMachine(name='root'" in rendered
    assert "description: Top level" in rendered
    assert "text blocks:" in rendered
    assert "hello\\nworld" in rendered
    assert "container transitions:" not in rendered

    root.remove_text_block(note)
    assert root.text_blocks == []


def test_concurrence_operations_update_outcome_rules_layout_and_defaults():
    cc = Concurrence(
        name="cc",
        description="Parallel block",
        outcomes=[Outcome("done"), Outcome("failed")],
        default_outcome="done",
    )
    worker = make_leaf("worker", ["ok", "fail"])
    helper = make_leaf("helper", ["ready"])
    note = TextBlock(x=1.0, y=2.0, content="parallel")

    cc.add_state(worker)
    cc.add_state(helper)
    cc.add_text_block(note)
    cc.layout.set_state_position("helper", 3.0, 4.0)
    cc.layout.set_primary_outcome_position("done", 5.0, 6.0)
    cc.layout.materialize_primary_outcome_position("done")
    cc.set_outcome_rule("done", "worker", "ok")
    cc.set_outcome_rule("done", "helper", "ready")
    cc.set_outcome_rule("failed", "worker", "fail")

    cc.rename_state("helper", "assistant")
    assert cc.get_state("assistant") is helper
    assert cc.layout.get_state_position("assistant").y == 4.0
    assert cc.outcome_map["done"]["assistant"] == ["ready"]

    cc.rename_child_state_outcome("worker", "ok", "success")
    assert cc.outcome_map["done"]["worker"] == ["success"]

    cc.rename_outcome("done", "finished")
    assert cc.default_outcome == "finished"
    assert "finished" in cc.outcome_map
    assert cc.layout.get_outcome_position("finished").x == 5.0

    cc.remove_outcome_rule("finished", "assistant")
    assert "assistant" not in cc.outcome_map["finished"]

    cc.remove_state("worker")
    assert cc.get_state("worker") is None
    assert "failed" not in cc.outcome_map

    cc.remove_outcome("finished")
    assert cc.default_outcome is None
    assert cc.layout.get_outcome_position("finished") is None

    rendered = str(cc)
    assert "Concurrence(name='cc'" in rendered
    assert "description: Parallel block" in rendered
    assert "text blocks:" in rendered
    assert "parallel" in rendered


def test_concurrence_outcome_rules_preserve_multiple_state_outcomes():
    cc = Concurrence(
        name="cc",
        outcomes=[Outcome("finished")],
        default_outcome="finished",
    )
    worker = make_leaf("worker", ["ok", "retry", "failed"])

    cc.add_state(worker)
    cc.set_outcome_rule("finished", "worker", "ok")
    cc.set_outcome_rule("finished", "worker", "retry")
    cc.set_outcome_rule("finished", "worker", "ok")

    assert cc.outcome_map["finished"]["worker"] == ["ok", "retry"]

    cc.rename_child_state_outcome("worker", "retry", "ok")
    assert cc.outcome_map["finished"]["worker"] == ["ok"]

    cc.set_outcome_rule("finished", "worker", "retry")
    cc.remove_outcome_rule("finished", "worker", "ok")
    assert cc.outcome_map["finished"]["worker"] == ["retry"]

    cc.remove_outcome_rule("finished", "worker", "retry")
    assert "finished" not in cc.outcome_map


def test_concurrence_xml_roundtrip_preserves_multiple_state_outcomes_per_final_outcome():
    root = StateMachine(name="root", outcomes=[Outcome("done")], start_state="cc")
    cc = Concurrence(
        name="cc",
        outcomes=[Outcome("finished")],
        default_outcome="finished",
    )
    worker = make_leaf("worker", ["ok", "retry"])

    cc.add_state(worker)
    cc.set_outcome_rule("finished", "worker", "ok")
    cc.set_outcome_rule("finished", "worker", "retry")
    root.add_state(cc)
    root.add_transition("cc", Transition("finished", "done"))

    xml_text = model_to_xml(root)
    loaded_root = model_from_xml(xml_text)
    loaded_cc = loaded_root.get_state("cc")

    assert loaded_cc is not None
    assert loaded_cc.outcome_map["finished"]["worker"] == ["ok", "retry"]
    assert xml_text.count('<Item state="worker" outcome="ok" />') == 1
    assert xml_text.count('<Item state="worker" outcome="retry" />') == 1
