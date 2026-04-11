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
"""Canonical dialog exports and compatibility wrappers for editor dialogs."""

from importlib import import_module

__all__ = [
    "BlackboardKeyDialog",
    "ConcurrenceDialog",
    "OutcomeDescriptionDialog",
    "ParameterOverwriteDialog",
    "StateMachineDialog",
    "StatePropertiesDialog",
]

_DIALOG_MODULES = {
    "BlackboardKeyDialog": "yasmin_editor.editor_gui.dialogs.blackboard_key_dialog",
    "ConcurrenceDialog": "yasmin_editor.editor_gui.dialogs.concurrence_dialog",
    "OutcomeDescriptionDialog": "yasmin_editor.editor_gui.dialogs.outcome_description_dialog",
    "ParameterOverwriteDialog": "yasmin_editor.editor_gui.dialogs.parameter_overwrite_dialog",
    "StateMachineDialog": "yasmin_editor.editor_gui.dialogs.state_machine_dialog",
    "StatePropertiesDialog": "yasmin_editor.editor_gui.dialogs.state_properties_dialog",
}


def __getattr__(name: str):
    module_name = _DIALOG_MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_name)
    return getattr(module, name)
