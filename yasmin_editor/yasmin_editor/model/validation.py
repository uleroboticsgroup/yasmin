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

from yasmin_editor.dataclass_compat import dataclass, field
from typing import Iterable, List, Set, Union

from .container_state import ContainerState, iter_outcome_rule_values
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

    errors: List[ValidationMessage] = field(default_factory=list)
    warnings: List[ValidationMessage] = field(default_factory=list)

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

        lines: List[str] = []

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
    parent_targets: Union[Set[str], None],
) -> None:
    """Validate one state recursively."""

    _validate_common_state_fields(state, result, path)

    if isinstance(state, StateMachine):
        _validate_state_machine(state, result, path, parent_targets)
    elif isinstance(state, ContainerState):
        _validate_container_state(state, result, path, parent_targets)
    else:
        _validate_leaf_state(state, result, path)


def _validate_unique_named_items(
    items: Iterable[object],
    *,
    path: str,
    result: ValidationResult,
    field_name: str,
) -> None:
    """Validate that a sequence of named model items is non-empty and unique."""

    seen_names: Set[str] = set()

    for item in items:
        name = getattr(item, "name", "")
        if not name:
            result.add_error(path, f"{field_name} name must not be empty")
            continue

        if name in seen_names:
            result.add_error(path, f"Duplicate {field_name.lower()} '{name}'")
        seen_names.add(name)


def _validate_common_state_fields(
    state: State,
    result: ValidationResult,
    path: str,
) -> None:
    """Validate fields common to all states."""

    if not state.name:
        result.add_error(path, "State name must not be empty")

    _validate_unique_named_items(
        state.outcomes,
        path=path,
        result=result,
        field_name="Outcome",
    )
    _validate_unique_named_items(
        state.keys,
        path=path,
        result=result,
        field_name="Key",
    )


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


def _validate_conflicting_container_names(
    *,
    path: str,
    result: ValidationResult,
    state_names: Set[str],
    outcome_names: Set[str],
) -> None:
    """Validate that container child states and final outcomes do not collide."""

    conflicting_names = sorted(state_names & outcome_names)
    for name in conflicting_names:
        result.add_error(
            path,
            f"Name '{name}' is used by both a child state and a final outcome",
        )


def _validate_child_state_binding(
    *,
    container_path: str,
    result: ValidationResult,
    state_name: str,
    child_state: State,
) -> str:
    """Validate one child-state dictionary binding and return its child path."""

    child_path = f"{container_path}/{state_name}"

    if not child_state.name:
        result.add_error(child_path, "Child state name must not be empty")
    elif child_state.name != state_name:
        result.add_error(
            child_path,
            f"Dictionary key '{state_name}' does not match state name '{child_state.name}'",
        )

    return child_path


def _validate_state_machine(
    state_machine: StateMachine,
    result: ValidationResult,
    path: str,
    parent_targets: Union[Set[str], None],
) -> None:
    """Validate a state machine recursively."""

    if not state_machine.states:
        result.add_warning(path, "State machine has no child states")

    state_names = set(state_machine.states)
    outcome_names = {outcome.name for outcome in state_machine.outcomes}
    _validate_conflicting_container_names(
        path=path,
        result=result,
        state_names=state_names,
        outcome_names=outcome_names,
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
        child_path = _validate_child_state_binding(
            container_path=path,
            result=result,
            state_name=state_name,
            child_state=child_state,
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


def _validate_container_state(
    container: ContainerState,
    result: ValidationResult,
    path: str,
    parent_targets: Union[Set[str], None],
) -> None:
    """Validate a container state (Concurrence or OrthogonalState) recursively."""

    kind = (
        "Orthogonal state"
        if container._container_name == "OrthogonalState"
        else "Concurrence"
    )
    if not container.states:
        result.add_warning(path, f"{kind} has no child states")

    state_names = set(container.states)
    outcome_names = {outcome.name for outcome in container.outcomes}
    _validate_conflicting_container_names(
        path=path,
        result=result,
        state_names=state_names,
        outcome_names=outcome_names,
    )
    nested_parent_targets = state_names | outcome_names | (parent_targets or set())

    if not outcome_names:
        result.add_error(path, f"{kind} requires at least one outcome")

    if container.default_outcome and container.default_outcome not in outcome_names:
        result.add_error(
            path,
            f"Default outcome '{container.default_outcome}' does not exist",
        )

    for state_name, child_state in container.states.items():
        child_path = _validate_child_state_binding(
            container_path=path,
            result=result,
            state_name=state_name,
            child_state=child_state,
        )

        _validate_state(child_state, result, child_path, nested_parent_targets)

    for outcome_name, mapping in container.outcome_map.items():
        if outcome_name not in outcome_names:
            result.add_error(
                path,
                f"Outcome map references unknown outcome '{outcome_name}'",
            )

        for state_name, state_outcomes in mapping.items():
            if state_name not in state_names:
                result.add_error(
                    path,
                    f"Outcome map references unknown state '{state_name}'",
                )
                continue

            child_state = container.states[state_name]
            child_outcomes = {outcome.name for outcome in child_state.outcomes}
            for state_outcome in iter_outcome_rule_values(state_outcomes):
                if child_outcomes and state_outcome not in child_outcomes:
                    result.add_warning(
                        f"{path}/{state_name}",
                        f"Outcome map references unknown outcome '{state_outcome}'",
                    )
