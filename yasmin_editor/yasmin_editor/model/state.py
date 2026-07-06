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

from typing import List, Dict, Union
from yasmin_editor.dataclass_compat import dataclass, field

from .key import Key
from .outcome import Outcome
from .parameter import Parameter


@dataclass(slots=True, repr=False)
class State:
    """Base model for all YASMIN states."""

    name: str
    description: str = ""
    keys: List[Key] = field(default_factory=list)
    outcomes: List[Outcome] = field(default_factory=list)
    parameters: List[Parameter] = field(default_factory=list)
    remappings: Dict[str, str] = field(default_factory=dict)
    parameter_mappings: Dict[str, str] = field(default_factory=dict)
    state_type: Union[str, None] = None
    module: Union[str, None] = None
    class_name: Union[str, None] = None
    package_name: Union[str, None] = None
    file_name: Union[str, None] = None

    def add_key(self, key: Key) -> None:
        """Add a blackboard key to the state."""
        self.keys.append(key)

    def add_outcome(self, outcome: Outcome) -> None:
        """Add an outcome to the state."""
        self.outcomes.append(outcome)

    def add_parameter(self, parameter: Parameter) -> None:
        """Add a declared parameter to the state."""
        self.parameters.append(parameter)

    def get_outcome(self, name: str) -> Union[Outcome, None]:
        """Return an outcome by name."""
        for outcome in self.outcomes:
            if outcome.name == name:
                return outcome
        return None

    def rename_outcome(self, old_name: str, new_name: str) -> None:
        """Rename one outcome of this state."""
        if old_name == new_name:
            return
        outcome = self.get_outcome(old_name)
        if outcome is None:
            return
        if self.get_outcome(new_name) is not None:
            raise ValueError(f"Outcome '{new_name}' already exists")
        outcome.name = new_name

    @property
    def is_container(self) -> bool:
        """Return whether this state contains child states."""
        return False

    @property
    def is_leaf(self) -> bool:
        """Return whether this state is a leaf state."""
        return not self.is_container

    def _format_header(self) -> str:
        """Return a compact one-line header for this state."""
        parts: List[str] = [self.name]
        meta: List[str] = []
        if self.state_type:
            meta.append(f"type={self.state_type}")
        if self.class_name:
            meta.append(f"class={self.class_name}")
        if self.module:
            meta.append(f"module={self.module}")
        if self.package_name:
            meta.append(f"package={self.package_name}")
        if self.file_name:
            meta.append(f"file={self.file_name}")
        if meta:
            parts.append(f"({', '.join(meta)})")
        return " ".join(parts)

    def to_string(self, indent: int = 0) -> str:
        """Return a human-readable representation of the state."""
        prefix = " " * indent
        lines = [f"{prefix}State({self._format_header()})"]
        if self.outcomes:
            lines.append(
                f"{prefix}  outcomes: {', '.join(outcome.name for outcome in self.outcomes)}"
            )
        if self.parameters:
            lines.append(
                f"{prefix}  params: {', '.join(parameter.name for parameter in self.parameters)}"
            )
        if self.keys:
            lines.append(f"{prefix}  keys: {', '.join(key.name for key in self.keys)}")
        if self.parameter_mappings:
            lines.append(
                f"{prefix}  param remap: "
                + ", ".join(
                    f"{source}->{target}"
                    for source, target in self.parameter_mappings.items()
                )
            )
        if self.remappings:
            lines.append(
                f"{prefix}  remap: "
                + ", ".join(
                    f"{source}->{target}" for source, target in self.remappings.items()
                )
            )
        if self.description:
            lines.append(f"{prefix}  description: {self.description}")
        return "\n".join(lines)

    def __str__(self) -> str:
        """Return a compact human-readable representation."""
        return self.to_string()

    __repr__ = __str__
