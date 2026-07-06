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
from typing import List, Dict, Tuple, Set, Union
from yasmin_editor.dataclass_compat import dataclass, field

from .layout import Layout
from .state import State
from .text_block import TextBlock
from .transition import Transition


@dataclass(slots=True, repr=False)
class StateMachine(State):
    """Represents a YASMIN state machine container."""

    start_state: Union[str, None] = None
    states: Dict[str, State] = field(default_factory=dict)
    transitions: Dict[str, List[Transition]] = field(default_factory=dict)
    layout: Layout = field(default_factory=Layout)
    text_blocks: List[TextBlock] = field(default_factory=list)

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""
        return True

    def _assert_child_name_available(
        self,
        name: str,
        *,
        exclude_state: Union[str, None] = None,
        exclude_outcome: Union[str, None] = None,
    ) -> None:
        """Validate that a child state or final outcome name is available."""
        if name in self.states and name != exclude_state:
            raise ValueError(
                f"Name '{name}' is already used by a child state in this state machine"
            )
        if any(
            outcome.name == name and outcome.name != exclude_outcome
            for outcome in self.outcomes
        ):
            raise ValueError(
                f"Name '{name}' is already used by a final outcome in this state machine"
            )

    def add_state(self, state: State) -> None:
        """Add a child state to the state machine."""
        self._assert_child_name_available(state.name)
        self.states[state.name] = state
        self.transitions.setdefault(state.name, [])

    def add_outcome(self, outcome) -> None:
        """Add a final outcome to the state machine."""
        self._assert_child_name_available(outcome.name)
        State.add_outcome(self, outcome)

    def add_text_block(self, text_block: TextBlock) -> None:
        """Add a free-form text block to the container."""
        self.text_blocks.append(text_block)

    def remove_text_block(self, text_block: TextBlock) -> None:
        """Remove one free-form text block from the container."""
        self.text_blocks = [item for item in self.text_blocks if item is not text_block]

    def add_transition(self, state_name: str, transition: Transition) -> None:
        """Add a transition for a child state."""
        transition_list = self.transitions.setdefault(state_name, [])
        for item in transition_list:
            if (
                item.source_outcome == transition.source_outcome
                and item.target == transition.target
            ):
                item.target_instance_id = transition.target_instance_id
                return
        transition_list.append(transition)

    def remove_transition(
        self,
        state_name: str,
        source_outcome: str,
        target: str,
    ) -> None:
        """Remove a transition for a child state."""
        transitions = self.transitions.get(state_name, [])
        self.transitions[state_name] = [
            item
            for item in transitions
            if not (item.source_outcome == source_outcome and item.target == target)
        ]
        if not self.transitions.get(state_name):
            self.transitions.pop(state_name, None)

    def remove_state(self, name: str) -> None:
        """Remove a child state and all related transitions."""
        self.states.pop(name, None)
        self.transitions.pop(name, None)
        for transitions in self.transitions.values():
            transitions[:] = [item for item in transitions if item.target != name]
        if self.start_state == name:
            self.start_state = None
        self.layout.remove_state_position(name)

    def rename_transition_owner(self, old_owner: str, new_owner: str) -> None:
        """Rename one transition owner entry inside this container."""
        if old_owner == new_owner:
            return
        old_transitions = self.transitions.pop(old_owner, [])
        if not old_transitions:
            return
        merged = self.transitions.setdefault(new_owner, [])
        for transition in old_transitions:
            if any(
                item.source_outcome == transition.source_outcome
                and item.target == transition.target
                for item in merged
            ):
                continue
            merged.append(transition)

    def rename_state(self, old_name: str, new_name: str) -> None:
        """Rename a child state and update all related references."""
        if old_name == new_name:
            return
        self._assert_child_name_available(new_name, exclude_state=old_name)
        state = self.states.pop(old_name)
        state.name = new_name
        self.states[new_name] = state
        self.rename_transition_owner(old_name, new_name)
        for transitions in self.transitions.values():
            for transition in transitions:
                if transition.target == old_name:
                    transition.target = new_name
        if self.start_state == old_name:
            self.start_state = new_name
        self.layout.rename_state_position(old_name, new_name)

    def rename_outcome(self, old_name: str, new_name: str) -> None:
        """Rename a final outcome and update all related references."""
        if old_name == new_name:
            return
        if self.get_outcome(old_name) is None:
            return
        self._assert_child_name_available(new_name, exclude_outcome=old_name)
        State.rename_outcome(self, old_name, new_name)
        for transitions in self.transitions.values():
            for transition in transitions:
                if transition.target == old_name:
                    transition.target = new_name
        self.layout.rename_outcome_position(old_name, new_name)

    def rename_child_state_outcome(
        self,
        state_name: str,
        old_outcome: str,
        new_outcome: str,
    ) -> None:
        """Rename one outcome referenced by transitions of a child state."""
        if old_outcome == new_outcome:
            return
        transitions = self.transitions.get(state_name, [])
        if not transitions:
            return
        deduplicated: List[Transition] = []
        seen: Set[Tuple[str, str]] = set()
        for transition in transitions:
            source_outcome = (
                new_outcome
                if transition.source_outcome == old_outcome
                else transition.source_outcome
            )
            key = (source_outcome, transition.target)
            if key in seen:
                continue
            seen.add(key)
            transition.source_outcome = source_outcome
            deduplicated.append(transition)
        if deduplicated:
            self.transitions[state_name] = deduplicated
        else:
            self.transitions.pop(state_name, None)

    def remove_outcome(self, name: str) -> None:
        """Remove a final outcome and all related transitions."""
        self.outcomes = [outcome for outcome in self.outcomes if outcome.name != name]
        self.transitions.pop(name, None)
        for transitions in self.transitions.values():
            transitions[:] = [item for item in transitions if item.target != name]
        self.layout.remove_outcome_position(name)

    def get_state(self, name: str) -> Union[State, None]:
        """Return a child state by name."""
        return self.states.get(name)

    def to_string(self, indent: int = 0) -> str:
        """Return a human-readable representation of the state machine."""
        prefix = " " * indent
        lines: List[str] = []
        header = f"{prefix}StateMachine(name={self.name!r}"
        if self.start_state:
            header += f", start_state={self.start_state!r}"
        if self.outcomes:
            header += ", outcomes=[{}]".format(
                ", ".join(outcome.name for outcome in self.outcomes)
            )
        if self.keys:
            header += ", keys=[{}]".format(", ".join(key.name for key in self.keys))
        header += ")"
        lines.append(header)
        if self.description:
            lines.append(f"{prefix}  description: {self.description}")
        if self.states:
            lines.append(f"{prefix}  states:")
            for state_name, state in self.states.items():
                if state.is_container:
                    lines.append(f"{prefix}    - {state.name}")
                    lines.append(state.to_string(indent + 8))
                else:
                    lines.append(f"{prefix}    - {state._format_header()}")
                state_transitions = self.transitions.get(state_name, [])
                for transition in state_transitions:
                    lines.append(
                        f"{prefix}      {transition.source_outcome} -> {transition.target}"
                    )
                if state.remappings:
                    lines.append(
                        f"{prefix}      remap: "
                        + ", ".join(
                            f"{source}->{target}"
                            for source, target in state.remappings.items()
                        )
                    )
        if self.text_blocks:
            lines.append(f"{prefix}  text blocks:")
            for text_block in self.text_blocks:
                lines.extend(text_block.to_text_block_lines(prefix))
        if self.outcomes:
            lines.append(f"{prefix}  final outcomes:")
            for outcome in self.outcomes:
                if outcome.description:
                    lines.append(f"{prefix}    - {outcome.name}: {outcome.description}")
                else:
                    lines.append(f"{prefix}    - {outcome.name}")
        container_transitions = self.transitions.get(self.name, [])
        if container_transitions:
            lines.append(f"{prefix}  container transitions:")
            for transition in container_transitions:
                lines.append(
                    f"{prefix}    {transition.source_outcome} -> {transition.target}"
                )
        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""
        return self.to_string()

    __repr__ = __str__
