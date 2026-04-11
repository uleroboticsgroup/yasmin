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

"""Lazy runtime package exports used by the editor and tests.

The runtime backend depends on PyQt, but several pure helper modules live in the
same package. Keeping the `Runtime` export lazy lets tests import helper modules
such as `yasmin_editor.runtime.traversal` in headless environments.
"""

from __future__ import annotations

__all__ = ["Runtime"]


def __getattr__(name: str):
    if name == "Runtime":
        from .runtime import Runtime

        return Runtime
    raise AttributeError(name)
