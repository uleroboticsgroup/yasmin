# Copyright (C) 2025 Miguel Ángel González Santamarta
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

from typing import TYPE_CHECKING

from yasmin_editor.editor_gui.model_adapter import (
    create_empty_model,
    load_model_into_editor,
)
from yasmin_editor.io.xml_converter import model_from_xml, model_to_xml

if TYPE_CHECKING:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


class XmlManager:
    def __init__(self, editor: "YasminEditor") -> None:
        self.editor = editor

    def _reset_editor_state(self) -> None:
        self.editor.canvas.scene.clear()
        self.editor.state_nodes.clear()
        self.editor.final_outcomes.clear()
        self.editor.connections.clear()
        self.editor.root_sm_name = ""
        self.editor.start_state = None
        self.editor.root_sm_name_edit.clear()
        self.editor.root_sm_description_edit.clear()
        self.editor._blackboard_keys = []
        self.editor._blackboard_key_metadata = {}
        self.editor.refresh_blackboard_keys_list()
        self.editor.update_start_state_combo()
        self.editor.model = create_empty_model()

    def load_from_xml(self, file_path: str) -> None:
        self._reset_editor_state()

        self.editor._suspend_model_sync = True
        try:
            self.editor.model = model_from_xml(file_path)
            load_model_into_editor(self.editor, self.editor.model)
        finally:
            self.editor._suspend_model_sync = False

        self.editor.sync_model_from_gui()

    def save_to_xml(self, file_path: str) -> None:
        self.editor.sync_model_from_gui()
        model_to_xml(self.editor.model, file_path)
