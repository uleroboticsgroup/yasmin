#!/usr/bin/env python3

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
"""Validation helpers for the YASMIN editor model."""

from __future__ import annotations

from dataclasses import dataclass, field

from .concurrence import Concurrence
from .state import State
from .state_machine import StateMachine


@dataclass(slots=True)
class ValidationMessage:
    """Represents one validation message."""

    path: str
    message: str


@dataclass(slots=True)
class ValidationResult:
    """Collects validation errors and warnings."""

    errors: list[ValidationMessage] = field(default_factory=list)
    warnings: list[ValidationMessage] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        """Return whether the validated model contains no errors."""

        return not self.errors

    def add_error(self, path: str, message: str) -> None:
        """Add one validation error."""

        self.errors.append(ValidationMessage(path=path, message=message))

    def add_warning(self, path: str, message: str) -> None:
        """Add one validation warning."""

        self.warnings.append(ValidationMessage(path=path, message=message))

    def extend(self, other: "ValidationResult") -> None:
        """Merge another validation result into this one."""

        self.errors.extend(other.errors)
        self.warnings.extend(other.warnings)

    def __str__(self) -> str:
        """Return a readable validation summary."""

        lines: list[str] = []

        if self.errors:
            lines.append("Errors:")
            for error in self.errors:
                lines.append(f"  - {error.path}: {error.message}")

        if self.warnings:
            if lines:
                lines.append("")
            lines.append("Warnings:")
            for warning in self.warnings:
                lines.append(f"  - {warning.path}: {warning.message}")

        if not lines:
            lines.append("Validation successful")

        return "\n".join(lines)

    __repr__ = __str__


def validate_model(model: State) -> ValidationResult:
    """Validate a model tree."""

    result = ValidationResult()
    _validate_state(model, result, model.name, parent_targets=None)
    return result


def _validate_state(
    state: State,
    result: ValidationResult,
    path: str,
    parent_targets: set[str] | None,
) -> None:
    """Validate one state recursively."""

    _validate_common_state_fields(state, result, path)

    if isinstance(state, StateMachine):
        _validate_state_machine(state, result, path, parent_targets)
    elif isinstance(state, Concurrence):
        _validate_concurrence(state, result, path, parent_targets)
    else:
        _validate_leaf_state(state, result, path)


def _validate_common_state_fields(
    state: State,
    result: ValidationResult,
    path: str,
) -> None:
    """Validate fields common to all states."""

    if not state.name:
        result.add_error(path, "State name must not be empty")

    outcome_names: set[str] = set()
    for outcome in state.outcomes:
        if not outcome.name:
            result.add_error(path, "Outcome name must not be empty")
            continue

        if outcome.name in outcome_names:
            result.add_error(path, f"Duplicate outcome '{outcome.name}'")
        outcome_names.add(outcome.name)

    key_names: set[str] = set()
    for key in state.keys:
        if not key.name:
            result.add_error(path, "Key name must not be empty")
            continue

        if key.name in key_names:
            result.add_error(path, f"Duplicate key '{key.name}'")
        key_names.add(key.name)


def _validate_leaf_state(state: State, result: ValidationResult, path: str) -> None:
    """Validate a leaf state."""

    if state.state_type == "py":
        if not state.module:
            result.add_error(path, "Python state requires 'module'")
        if not state.class_name:
            result.add_error(path, "Python state requires 'class_name'")

    elif state.state_type == "cpp":
        if not state.class_name:
            result.add_error(path, "C++ state requires 'class_name'")

    elif state.state_type == "xml":
        if not state.file_name:
            result.add_error(path, "XML state requires 'file_name'")
        if not state.package_name:
            result.add_warning(path, "XML state usually should define 'package_name'")

    elif state.state_type is None:
        result.add_warning(path, "Leaf state has no 'state_type'")


def _validate_state_machine(
    state_machine: StateMachine,
    result: ValidationResult,
    path: str,
    parent_targets: set[str] | None,
) -> None:
    """Validate a state machine recursively."""

    if not state_machine.states:
        result.add_warning(path, "State machine has no child states")

    state_names = set(state_machine.states.keys())
    outcome_names = {outcome.name for outcome in state_machine.outcomes}
    conflicting_names = sorted(state_names & outcome_names)
    for name in conflicting_names:
        result.add_error(
            path,
            f"Name '{name}' is used by both a child state and a final outcome",
        )
    local_targets = state_names | outcome_names
    nested_parent_targets = local_targets | (parent_targets or set())

    if not outcome_names:
        result.add_error(path, "State machine requires at least one outcome")

    if not state_machine.start_state:
        if len(state_machine.states) > 1:
            result.add_error(path, "State machine requires 'start_state'")
    elif state_machine.start_state not in state_names:
        result.add_error(
            path,
            f"Start state '{state_machine.start_state}' does not exist",
        )

    for state_name, child_state in state_machine.states.items():
        child_path = f"{path}/{state_name}"

        if not child_state.name:
            result.add_error(child_path, "Child state name must not be empty")
        elif child_state.name != state_name:
            result.add_error(
                child_path,
                f"Dictionary key '{state_name}' does not match state name '{child_state.name}'",
            )

        _validate_state(child_state, result, child_path, nested_parent_targets)

        transitions = state_machine.transitions.get(state_name, [])
        for transition in transitions:
            if transition.target not in local_targets:
                result.add_error(
                    child_path,
                    f"Transition target '{transition.target}' does not exist",
                )

        for source_key, target_key in child_state.remappings.items():
            if not source_key:
                result.add_error(child_path, "Remapping source key must not be empty")
            if not target_key:
                result.add_error(child_path, "Remapping target key must not be empty")

    for owner_name, transitions in state_machine.transitions.items():
        if owner_name == state_machine.name:
            for transition in transitions:
                if transition.source_outcome not in outcome_names:
                    result.add_warning(
                        path,
                        f"Container transition uses unknown outcome '{transition.source_outcome}'",
                    )
                if transition.target not in nested_parent_targets:
                    result.add_error(
                        path,
                        f"Container transition target '{transition.target}' does not exist",
                    )
            continue

        if owner_name in state_names:
            continue

        if owner_name in outcome_names:
            for transition in transitions:
                if transition.target not in nested_parent_targets:
                    result.add_error(
                        path,
                        f"Final outcome transition target '{transition.target}' does not exist",
                    )
            continue

        result.add_error(
            path,
            f"Transitions defined for unknown owner '{owner_name}'",
        )


def _validate_concurrence(
    concurrence: Concurrence,
    result: ValidationResult,
    path: str,
    parent_targets: set[str] | None,
) -> None:
    """Validate a concurrence recursively."""

    if not concurrence.states:
        result.add_warning(path, "Concurrence has no child states")

    state_names = set(concurrence.states.keys())
    outcome_names = {outcome.name for outcome in concurrence.outcomes}
    conflicting_names = sorted(state_names & outcome_names)
    for name in conflicting_names:
        result.add_error(
            path,
            f"Name '{name}' is used by both a child state and a final outcome",
        )
    nested_parent_targets = state_names | outcome_names | (parent_targets or set())

    if not outcome_names:
        result.add_error(path, "Concurrence requires at least one outcome")

    if concurrence.default_outcome and concurrence.default_outcome not in outcome_names:
        result.add_error(
            path,
            f"Default outcome '{concurrence.default_outcome}' does not exist",
        )

    for state_name, child_state in concurrence.states.items():
        child_path = f"{path}/{state_name}"

        if not child_state.name:
            result.add_error(child_path, "Child state name must not be empty")
        elif child_state.name != state_name:
            result.add_error(
                child_path,
                f"Dictionary key '{state_name}' does not match state name '{child_state.name}'",
            )

        _validate_state(child_state, result, child_path, nested_parent_targets)

    for outcome_name, mapping in concurrence.outcome_map.items():
        if outcome_name not in outcome_names:
            result.add_error(
                path,
                f"Outcome map references unknown outcome '{outcome_name}'",
            )

        for state_name, state_outcome in mapping.items():
            if state_name not in state_names:
                result.add_error(
                    path,
                    f"Outcome map references unknown state '{state_name}'",
                )
                continue

            child_state = concurrence.states[state_name]
            child_outcomes = {outcome.name for outcome in child_state.outcomes}
            if child_outcomes and state_outcome not in child_outcomes:
                result.add_warning(
                    f"{path}/{state_name}",
                    f"Outcome map references unknown outcome '{state_outcome}'",
                )
