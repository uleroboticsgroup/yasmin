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

from importlib import import_module

__all__ = ["OutcomeDescriptionDialog"]


def __getattr__(name: str):
    if name == "OutcomeDescriptionDialog":
        module = import_module(
            "yasmin_editor.editor_gui.dialogs.outcome_description_dialog"
        )
        return module.OutcomeDescriptionDialog
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
