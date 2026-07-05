# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from .outcome import Outcome
from .state import State


@dataclass(slots=True, repr=False)
class JoinState(State):
    """Represents a YASMIN join-state leaf that synchronizes region threads."""

    sync_id: str = ""
    join_outcome: str = "joined"

    def __post_init__(self) -> None:
        if not self.outcomes:
            self.outcomes = [Outcome(name=self.join_outcome)]

    def __str__(self) -> str:
        parts = [f"JoinState({self.name})"]
        if self.sync_id:
            parts.append(f"sync_id={self.sync_id}")
        parts.append(f"outcome={self.join_outcome}")
        return " ".join(parts)

    __repr__ = __str__
