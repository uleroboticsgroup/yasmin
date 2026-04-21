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
"""Application bootstrap helpers for the YASMIN editor."""

from __future__ import annotations

import argparse
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PyQt5.QtWidgets import QApplication
    from yasmin_plugins_manager.plugin_manager import PluginManager
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def parse_cli_args(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    """Parse editor-specific CLI arguments while preserving Qt arguments."""

    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--xml-file",
        dest="xml_file",
        default="",
        help="Path to a YASMIN XML state machine to open at startup.",
    )
    return parser.parse_known_args(argv)


def viewport_is_ready(viewport: object, min_size: int = 10) -> bool:
    """Return whether a canvas viewport is large enough for first paint."""

    return viewport.width() >= min_size and viewport.height() >= min_size


def build_qt_argv(program_name: str, qt_args: list[str]) -> list[str]:
    """Build the QApplication argument vector.

    The editor only consumes its own CLI flags. Every unknown argument is passed
    through unchanged so Qt keeps handling style and platform flags exactly like
    in the original startup flow.
    """

    return [program_name, *qt_args]


def startup_open_message(xml_file: str) -> str:
    """Return the status-bar message used after opening a startup XML file."""

    return f"Opened: {xml_file}"


def startup_error_message(exc: Exception) -> str:
    """Return the startup error dialog message for XML loading failures."""

    return f"Failed to open startup XML file: {exc}"


def wait_for_canvas_ready(
    editor: "YasminEditor",
    timeout_sec: float = 3.0,
    min_size: int = 10,
) -> bool:
    from PyQt5.QtWidgets import QApplication

    end_time = time.monotonic() + timeout_sec

    while time.monotonic() < end_time:
        QApplication.processEvents()
        if viewport_is_ready(editor.canvas.viewport(), min_size=min_size):
            return True
        time.sleep(0.05)

    return False


def create_application(argv: list[str]) -> "QApplication":
    from PyQt5.QtWidgets import QApplication

    return QApplication(argv)


def create_plugin_manager() -> "PluginManager":
    from yasmin_plugins_manager.plugin_manager import PluginManager

    manager = PluginManager()
    manager.load_all_plugins()
    return manager


def open_startup_xml(editor: "YasminEditor", xml_file: str) -> None:
    """Open the startup XML file after the editor canvas is ready."""

    if not xml_file:
        return

    from PyQt5.QtWidgets import QMessageBox

    try:
        wait_for_canvas_ready(editor)
        editor.load_from_xml(xml_file)
        editor.statusBar().showMessage(startup_open_message(xml_file), 3000)
    except Exception as exc:
        QMessageBox.critical(
            editor,
            "Error",
            startup_error_message(exc),
        )


def run_editor(argv: list[str] | None = None) -> int:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor

    raw_argv = list(sys.argv if argv is None else argv)
    args, unknown = parse_cli_args(raw_argv[1:])

    manager = create_plugin_manager()
    app = create_application(build_qt_argv(raw_argv[0], unknown))

    editor = YasminEditor(manager)
    editor.show_startup_window()
    open_startup_xml(editor, args.xml_file)
    exit_code = app.exec_()

    # Cleanup plugin manager to release ROS 2 nodes and other resources
    try:
        # Destroy the singleton YasminNode instance to clean up ROS 2 resources
        from yasmin_ros.yasmin_node import YasminNode

        YasminNode.destroy_instance()
    except Exception:
        pass

    del manager

    return exit_code
