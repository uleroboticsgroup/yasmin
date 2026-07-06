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

import argparse
import sys
import time
from typing import TYPE_CHECKING, List, Tuple, Union

from yasmin_editor.qt_compat import exec_app

if TYPE_CHECKING:
    from yasmin_editor.qt_compat import QtWidgets
    from yasmin_plugins_manager.plugin_manager import PluginManager
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor


def parse_cli_args(
    argv: Union[List[str], None] = None,
) -> Tuple[argparse.Namespace, List[str]]:
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


def build_qt_argv(program_name: str, qt_args: List[str]) -> List[str]:
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
    from yasmin_editor.qt_compat import QtWidgets

    end_time = time.monotonic() + timeout_sec

    while time.monotonic() < end_time:
        QtWidgets.QApplication.processEvents()
        if viewport_is_ready(editor.canvas.viewport(), min_size=min_size):
            return True
        time.sleep(0.05)

    return False


def create_application(argv: List[str]) -> "QtWidgets.QApplication":
    from yasmin_editor.qt_compat import QtWidgets

    return QtWidgets.QApplication(argv)


def create_plugin_manager() -> "PluginManager":
    from yasmin_plugins_manager.plugin_manager import PluginManager

    manager = PluginManager()
    manager.load_all_plugins()
    return manager


def open_startup_xml(editor: "YasminEditor", xml_file: str) -> None:
    """Open the startup XML file after the editor canvas is ready."""

    if not xml_file:
        return

    from yasmin_editor.qt_compat import QtWidgets

    try:
        wait_for_canvas_ready(editor)
        editor.load_from_xml(xml_file)
        editor.statusBar().showMessage(startup_open_message(xml_file), 3000)
    except Exception as exc:
        QtWidgets.QMessageBox.critical(
            editor,
            "Error",
            startup_error_message(exc),
        )


def run_editor(argv: Union[List[str], None] = None) -> int:
    from yasmin_editor.editor_gui.yasmin_editor import YasminEditor

    raw_argv = list(sys.argv if argv is None else argv)
    args, unknown = parse_cli_args(raw_argv[1:])

    manager = create_plugin_manager()
    app = create_application(build_qt_argv(raw_argv[0], unknown))

    editor = YasminEditor(manager)
    editor.show_startup_window()
    open_startup_xml(editor, args.xml_file)
    exit_code = exec_app(app)

    # Cleanup plugin manager to release ROS 2 nodes and other resources
    try:
        # Destroy the singleton YasminNode instance to clean up ROS 2 resources
        from yasmin_ros.yasmin_node import YasminNode

        YasminNode.destroy_instance()
    except Exception:
        pass

    del manager

    return exit_code
