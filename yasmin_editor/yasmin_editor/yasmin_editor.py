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

import argparse
import sys
import time

from PyQt5.QtWidgets import QApplication, QMessageBox
from yasmin_editor.editor_gui.yasmin_editor import YasminEditor

from yasmin_plugins_manager.plugin_manager import PluginManager


def parse_args():
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--xml-file",
        dest="xml_file",
        default="",
        help="Path to a YASMIN XML state machine to open at startup.",
    )
    args, unknown = parser.parse_known_args()
    return args, unknown


def wait_for_canvas_ready(
    editor: YasminEditor, timeout_sec: float = 3.0, min_size: int = 10
) -> bool:
    end_time = time.monotonic() + timeout_sec

    while time.monotonic() < end_time:
        QApplication.processEvents()

        viewport = editor.canvas.viewport()
        if viewport.width() >= min_size and viewport.height() >= min_size:
            return True

        time.sleep(0.05)

    return False


def main() -> int:
    args, unknown = parse_args()

    manager = PluginManager()
    manager.load_all_plugins()

    qt_argv = [sys.argv[0]] + unknown
    app = QApplication(qt_argv)

    editor = YasminEditor(manager)
    editor.show()

    if args.xml_file:
        try:
            wait_for_canvas_ready(editor)
            editor.xml_manager.load_from_xml(args.xml_file)
            editor.statusBar().showMessage(f"Opened: {args.xml_file}", 3000)
        except Exception as e:
            QMessageBox.critical(
                editor,
                "Error",
                f"Failed to open startup XML file: {str(e)}",
            )

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
