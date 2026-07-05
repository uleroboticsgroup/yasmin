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

from yasmin_editor.dataclass_compat import dataclass
from .container_state import ContainerState


@dataclass(slots=True, repr=False)
class Concurrence(ContainerState):
    """Represents a YASMIN concurrence container."""

    @property
    def _container_name(self) -> str:
        return "Concurrence"

    @property
    def _child_label(self) -> str:
        return "states"

    @property
    def _child_term(self) -> str:
        return "child state"
