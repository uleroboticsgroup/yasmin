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

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class TextBlock:
    """Represents a free-form text annotation inside a container."""

    x: float = 0.0
    y: float = 0.0
    content: str = ""

    def to_text_block_lines(self, prefix: str = "") -> list[str]:
        """Return lines for displaying this text block in to_string output."""
        lines: list[str] = []
        preview = self.content.replace("\n", "\\n")
        lines.append(f"{prefix}    - ({self.x:.2f}, {self.y:.2f}): {preview}")
        return lines
