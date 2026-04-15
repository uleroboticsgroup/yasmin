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

"""Tests for parameter declaration and overwrite helper functions.

These tests focus on the editor-facing parameter normalization rules and on how
parent container declarations are updated when child overwrite rows change.
"""

from yasmin_editor.editor_gui.model_parameters import (
    apply_parameter_overwrites,
    dicts_to_parameters,
    get_parameter_overwrites_for_child,
    iter_declared_parameters,
    iter_valid_overwrite_pairs,
    normalize_parameter_name,
    parameters_to_dicts,
)
from yasmin_editor.model.parameter import Parameter
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine


def test_parameter_conversion_helpers_normalize_editor_rows_and_defaults():
    """Parameter rows should round-trip without keeping empty declarations."""

    parameters = [
        Parameter(
            name="speed",
            description="linear speed",
            default_type="float",
            default_value=0.5,
        ),
        Parameter(name="mode"),
    ]
    as_dicts = parameters_to_dicts(parameters)
    assert as_dicts == [
        {
            "name": "speed",
            "description": "linear speed",
            "default_type": "float",
            "default_value": 0.5,
            "has_default": True,
        },
        {
            "name": "mode",
            "description": "",
            "default_type": "",
            "default_value": None,
            "has_default": False,
        },
    ]

    restored = dicts_to_parameters(
        [
            {
                "name": "  timeout  ",
                "description": "  retries ",
                "default_type": " int ",
                "default_value": 3,
            },
            {"name": "   "},
        ]
    )
    assert restored == [
        Parameter(
            name="timeout", description="retries", default_type="int", default_value=3
        )
    ]
    assert normalize_parameter_name(None) == ""
    assert normalize_parameter_name("  speed  ") == "speed"


def test_parameter_overwrite_queries_include_parent_metadata():
    """Child overwrite rows should be enriched with the parent declaration data."""

    container = StateMachine(
        name="root",
        parameters=[
            Parameter(
                name="speed",
                description="linear speed",
                default_type="float",
                default_value=0.5,
            ),
            Parameter(
                name="mode",
                description="controller mode",
                default_type="str",
                default_value="auto",
            ),
        ],
    )
    child = State(
        name="worker", parameter_mappings={"local_speed": "speed", "profile": "missing"}
    )

    overwrites = get_parameter_overwrites_for_child(container, child)
    assert overwrites == [
        {
            "name": "speed",
            "child_parameter": "local_speed",
            "description": "linear speed",
            "default_type": "float",
            "default_value": 0.5,
        },
        {
            "name": "missing",
            "child_parameter": "profile",
            "description": "",
            "default_type": "",
            "default_value": "",
        },
    ]


def test_apply_parameter_overwrites_updates_child_mappings_and_prunes_unused_parent_parameters():
    """Applying overwrite rows should preserve order and remove unused declarations."""

    first_child = State(name="worker", parameter_mappings={"speed_local": "speed"})
    second_child = State(name="helper", parameter_mappings={"mode_local": "mode"})
    container = StateMachine(
        name="root",
        parameters=[
            Parameter(
                name="speed",
                description="linear speed",
                default_type="float",
                default_value=0.5,
            ),
            Parameter(
                name="mode",
                description="controller mode",
                default_type="str",
                default_value="auto",
            ),
            Parameter(
                name="unused",
                description="remove me",
                default_type="int",
                default_value=1,
            ),
        ],
        states={"worker": first_child, "helper": second_child},
    )

    apply_parameter_overwrites(
        container,
        first_child,
        [
            {
                "name": "mode",
                "child_parameter": "profile",
                "description": "controller mode",
                "default_type": "str",
                "default_value": "manual",
            },
            {
                "name": "new_param",
                "child_parameter": "threshold",
                "description": "fresh declaration",
                "default_type": "float",
                "default_value": 0.1,
            },
        ],
    )

    assert first_child.parameter_mappings == {"profile": "mode", "threshold": "new_param"}
    assert [
        (parameter.name, parameter.default_value) for parameter in container.parameters
    ] == [
        ("mode", "manual"),
        ("new_param", 0.1),
    ]


def test_parameter_overwrite_iterators_ignore_incomplete_rows():
    """Iterator helpers should keep only valid mappings and declarations."""

    overwrites = [
        {
            "name": "speed",
            "child_parameter": "local_speed",
            "description": "speed",
            "default_type": "float",
            "default_value": 1.0,
        },
        {"name": "", "child_parameter": "ignored"},
        {"name": "mode", "child_parameter": ""},
    ]

    assert iter_valid_overwrite_pairs(overwrites) == [("local_speed", "speed")]
    assert iter_declared_parameters(overwrites) == [
        Parameter(
            name="speed", description="speed", default_type="float", default_value=1.0
        ),
        Parameter(name="mode", description="", default_type="", default_value=None),
    ]
