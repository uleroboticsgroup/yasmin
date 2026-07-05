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

from typing import List
from yasmin_editor.dataclass_compat import dataclass


@dataclass(slots=True)
class TextBlock:
    """Represents a free-form text annotation inside a container."""

    x: float = 0.0
    y: float = 0.0
    content: str = ""

    def to_text_block_lines(self, prefix: str = "") -> List[str]:
        """Return lines for displaying this text block in to_string output."""
        lines: List[str] = []
        preview = self.content.replace("\n", "\\n")
        lines.append(f"{prefix}    - ({self.x:.2f}, {self.y:.2f}): {preview}")
        return lines
