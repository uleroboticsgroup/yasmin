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

"""Unit tests for child-name conflict checks and dialog result adapters."""

from yasmin_editor.editor_gui.child_name_conflicts import (
    has_final_outcome_name_conflict,
    has_state_name_conflict,
)
from yasmin_editor.editor_gui.dialog_result_adapters import (
    build_concurrence_kwargs,
    build_plugin_state_kwargs,
    build_state_machine_kwargs,
)


def test_state_name_conflict_detects_state_and_outcome_collisions():
    assert has_state_name_conflict(
        "foo",
        sibling_state_names=["foo", "bar"],
        sibling_outcome_names=["done"],
    )
    assert has_state_name_conflict(
        "done",
        sibling_state_names=["foo", "bar"],
        sibling_outcome_names=["done"],
    )
    assert not has_state_name_conflict(
        "foo",
        current_name="foo",
        sibling_state_names=["foo", "bar"],
        sibling_outcome_names=["done"],
    )


def test_final_outcome_name_conflict_respects_current_name():
    assert has_final_outcome_name_conflict(
        "worker",
        current_name=None,
        sibling_state_names=["worker"],
        sibling_outcome_names=["done"],
    )
    assert not has_final_outcome_name_conflict(
        "done",
        current_name="done",
        sibling_state_names=["worker"],
        sibling_outcome_names=["done"],
    )


def test_build_plugin_state_kwargs_maps_tuple_to_create_state_kwargs():
    plugin = object()
    result = build_plugin_state_kwargs(
        (
            "Foo",
            plugin,
            ["ok", "fail"],
            {"in": "bb_in"},
            "demo",
            [{"name": "rate", "value": 10}],
            [{"name": "rate", "value": 20}],
        )
    )

    assert result == {
        "name": "Foo",
        "plugin_info": plugin,
        "outcomes": ["ok", "fail"],
        "remappings": {"in": "bb_in"},
        "description": "demo",
        "defaults": [{"name": "rate", "value": 10}],
        "parameter_overwrites": [{"name": "rate", "value": 20}],
    }


def test_build_container_kwargs_for_state_machine_and_concurrence():
    state_machine_kwargs = build_state_machine_kwargs(
        (
            "Nested",
            ["done"],
            "start",
            {"input": "shared"},
            "nested machine",
            [{"name": "foo", "value": 1}],
        )
    )
    concurrence_kwargs = build_concurrence_kwargs(
        (
            "Parallel",
            ["done"],
            "done",
            {"input": "shared"},
            "parallel container",
            [{"name": "foo", "value": 1}],
        )
    )

    assert state_machine_kwargs == {
        "name": "Nested",
        "plugin_info": None,
        "is_state_machine": True,
        "is_concurrence": False,
        "outcomes": ["done"],
        "remappings": {"input": "shared"},
        "start_state": "start",
        "default_outcome": None,
        "description": "nested machine",
        "defaults": [{"name": "foo", "value": 1}],
    }
    assert concurrence_kwargs == {
        "name": "Parallel",
        "plugin_info": None,
        "is_state_machine": False,
        "is_concurrence": True,
        "outcomes": ["done"],
        "remappings": {"input": "shared"},
        "start_state": None,
        "default_outcome": "done",
        "description": "parallel container",
        "defaults": [{"name": "foo", "value": 1}],
    }
