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

"""Unit tests for pure blackboard metadata and remapping helpers.

These tests cover key normalization, derived key aggregation, remapping through
nested containers, and the persistence rules used by the blackboard sidebar.
"""

from types import SimpleNamespace

from yasmin_editor.editor_gui.blackboard_logic import (
    build_container_metadata_map,
    collect_blackboard_key_usage_for_model,
    collect_blackboard_key_usage_from_nodes,
    collect_container_key_lists,
    dicts_to_keys,
    format_blackboard_key_label,
    get_effective_blackboard_key_name,
    has_persistent_blackboard_metadata,
    keys_to_dicts,
    merge_container_keys,
    metadata_map_to_keys,
    should_hide_blackboard_key,
    state_uses_blackboard_key,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.key import Key
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine


class FakePluginInfo(SimpleNamespace):
    """Simple plugin metadata container used by the pure helper tests."""


def make_leaf(name: str, *, remappings=None, keys=None, parameter_mappings=None) -> State:
    """Create a small leaf state with the attributes needed by the tests."""

    return State(
        name=name,
        outcomes=[Outcome("done")],
        remappings=remappings or {},
        keys=keys or [],
        parameter_mappings=parameter_mappings or {},
    )


def test_blackboard_visibility_and_label_formatting_follow_sidebar_rules():
    """Hidden keys and text filters should behave like the sidebar helpers."""

    assert (
        should_hide_blackboard_key(
            item_text="shared_key (in)",
            key_name="shared_key",
            filter_text="SHARED",
            show_hidden=False,
        )
        is False
    )
    assert (
        should_hide_blackboard_key(
            item_text=".secret (in)",
            key_name=".secret",
            filter_text="",
            show_hidden=False,
        )
        is True
    )
    assert (
        should_hide_blackboard_key(
            item_text="shared_key (in)",
            key_name="shared_key",
            filter_text="missing",
            show_hidden=True,
        )
        is True
    )

    assert (
        format_blackboard_key_label(
            {
                "name": "shared_key",
                "key_type": "in/out",
                "default_type": "str",
                "default_value": "ready",
            }
        )
        == "shared_key (in/out) [default: ready, type: str]"
    )
    assert format_blackboard_key_label({"name": "generated", "key_type": "out"}) == (
        "generated (out)"
    )


def test_key_conversion_round_trip_normalizes_empty_rows_and_none_defaults():
    """Editor-table dictionaries should round-trip through the key model cleanly."""

    converted = dicts_to_keys(
        [
            {
                "name": "  alpha  ",
                "key_type": " in/out ",
                "description": "  shared ",
                "default_type": " str ",
                "default_value": "value",
            },
            {"name": "   "},
        ]
    )

    assert [key.name for key in converted] == ["alpha"]
    assert converted[0].key_type == "in/out"
    assert keys_to_dicts([Key(name="beta", key_type="out", default_value=None)]) == [
        {
            "name": "beta",
            "key_type": "out",
            "description": "",
            "default_type": "",
            "default_value": "",
        }
    ]


def test_metadata_helpers_build_sorted_keys_and_persistent_metadata_map():
    """Stored container metadata should be exported and sorted deterministically."""

    container = StateMachine(
        name="root",
        keys=[
            Key(name="beta", key_type="out", description="generated"),
            Key(name="alpha", key_type="in", default_type="int", default_value=5),
            Key(name=" "),
        ],
    )

    metadata_map = build_container_metadata_map(container)
    assert metadata_map == {
        "beta": {
            "description": "generated",
            "key_type": "out",
            "default_type": "",
            "default_value": "",
        },
        "alpha": {
            "description": "",
            "key_type": "in",
            "default_type": "int",
            "default_value": "5",
        },
    }

    sorted_keys = metadata_map_to_keys(metadata_map)
    assert [key.name for key in sorted_keys] == ["alpha", "beta"]
    assert has_persistent_blackboard_metadata(metadata_map["alpha"]) is True
    assert has_persistent_blackboard_metadata(metadata_map["beta"]) is False


def test_collect_blackboard_key_usage_from_nodes_merges_usage_with_metadata():
    """Visible node usage should merge input and output observations per key."""

    state_nodes = [
        SimpleNamespace(
            plugin_info=FakePluginInfo(
                input_keys=[{"name": "request", "description": "incoming"}],
                output_keys=[{"name": "result", "description": "generated"}],
            )
        ),
        SimpleNamespace(
            plugin_info=FakePluginInfo(
                input_keys=[{"name": "result", "description": "reused"}],
                output_keys=[],
            )
        ),
    ]
    metadata_map = {
        "request": {
            "description": "persistent input",
            "key_type": "in",
            "default_type": "str",
            "default_value": "hello",
        }
    }

    derived = collect_blackboard_key_usage_from_nodes(
        state_nodes,
        metadata_map,
        lambda _node, key_name: (
            f"effective_{key_name}" if key_name == "result" else key_name
        ),
    )

    assert derived == {
        "effective_result": {
            "name": "effective_result",
            "key_type": "in/out",
            "description": "generated",
            "default_type": "",
            "default_value": "",
        },
        "request": {
            "name": "request",
            "key_type": "in",
            "description": "persistent input",
            "default_type": "str",
            "default_value": "hello",
        },
    }


def test_collect_blackboard_key_usage_for_model_tracks_remaps_and_hidden_names():
    """Nested remappings should resolve to the final key name and hide intermediates."""

    nested = StateMachine(name="nested", remappings={"middle": "shared"})
    worker = make_leaf(name="worker", remappings={"raw": "middle"})
    nested.add_state(worker)

    root = StateMachine(
        name="root",
        keys=[
            Key(name="shared", key_type="in", default_type="str", default_value="ok"),
            Key(name="middle", key_type="in", description="transient"),
        ],
    )
    root.add_state(nested)

    resolver_calls = []

    def resolve_plugin_info(state_model: State):
        resolver_calls.append(state_model.name)
        return FakePluginInfo(
            input_keys=[{"name": "raw", "description": "input description"}],
            output_keys=[{"name": "raw", "description": "output description"}],
        )

    derived, hidden = collect_blackboard_key_usage_for_model(root, resolve_plugin_info)

    assert resolver_calls == ["worker"]
    assert derived == {
        "shared": {
            "name": "shared",
            "key_type": "in/out",
            "description": "input description",
            "default_type": "str",
            "default_value": "ok",
        }
    }
    assert hidden == {"raw", "middle"}
    assert get_effective_blackboard_key_name(
        [worker.remappings, nested.remappings], "raw"
    ) == ("shared")


def test_merge_container_keys_keeps_persistent_metadata_and_drops_hidden_intermediates():
    """Merge logic should preserve durable defaults but not leaked remap aliases."""

    root = StateMachine(
        name="root",
        keys=[
            Key(name="shared", key_type="in", default_type="str", default_value="ok"),
            Key(name="middle", key_type="in", description="transient alias"),
            Key(
                name="persistent_only", key_type="in", default_type="int", default_value=1
            ),
            Key(name="ephemeral", key_type="out", description="unused"),
        ],
    )
    nested = StateMachine(name="nested", remappings={"middle": "shared"})
    nested.add_state(make_leaf(name="worker", remappings={"raw": "middle"}))
    root.add_state(nested)

    def resolve_plugin_info(_state_model: State):
        return FakePluginInfo(input_keys=[{"name": "raw", "description": "from plugin"}])

    merged = merge_container_keys(root, resolve_plugin_info)

    assert merged == {
        "persistent_only": {
            "name": "persistent_only",
            "key_type": "in",
            "description": "",
            "default_type": "int",
            "default_value": "1",
        },
        "shared": {
            "name": "shared",
            "key_type": "in",
            "description": "from plugin",
            "default_type": "str",
            "default_value": "ok",
        },
    }


def test_state_uses_blackboard_key_and_collect_container_key_lists_handle_containers():
    """State usage checks and dialog key splits should respect remaps and key types."""

    nested = Concurrence(
        name="nested",
        keys=[
            Key(name="request", key_type="in", default_type="str", default_value="demo"),
            Key(name="shared", key_type="in/out", description="shared key"),
            Key(name="reply", key_type="out", description="result key"),
        ],
    )
    nested.add_state(make_leaf(name="worker", remappings={"raw": "request"}))

    def resolve_plugin_info(_state_model: State):
        return FakePluginInfo(
            input_keys=[{"name": "raw", "description": "needs request"}],
            output_keys=[{"name": "produced", "description": "unused"}],
        )

    assert state_uses_blackboard_key(nested, "request", resolve_plugin_info) is True
    assert state_uses_blackboard_key(nested, "missing", resolve_plugin_info) is False

    input_keys, output_keys = collect_container_key_lists(nested)
    assert [item["name"] for item in input_keys] == ["request", "shared"]
    assert [item["name"] for item in output_keys] == ["shared", "reply"]
    assert input_keys[0]["has_default"] is True
    assert output_keys[-1]["has_default"] is False
