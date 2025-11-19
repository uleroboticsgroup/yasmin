#!/usr/bin/env python3

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

import sys
from PyQt5.QtWidgets import QApplication
from yasmin_editor.plugins_manager.plugin_manager import PluginManager
from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def main() -> int:
    manager = PluginManager()
    manager.load_all_plugins()

    app = QApplication(sys.argv)
    editor = YasminEditor(manager)
    editor.show()

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
