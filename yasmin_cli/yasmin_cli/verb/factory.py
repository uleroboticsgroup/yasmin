from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

IGNORE_LIST = ["package.xml", "plugins.xml"]


def _strip_namespace(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _is_state_machine_xml(path: Path) -> bool:
    try:
        context = ET.iterparse(path, events=("start",))
        _, root = next(context)
        return _strip_namespace(root.tag) == "StateMachine"
    except (ET.ParseError, OSError, StopIteration):
        return False


def _xml_file_completer(prefix, parsed_args, **kwargs):
    current_dir = Path.cwd()
    matches: list[str] = []

    for path in sorted(current_dir.rglob("*.xml")):
        if path.name in IGNORE_LIST:
            continue

        if not _is_state_machine_xml(path):
            continue

        relative_path = path.relative_to(current_dir).as_posix()
        if relative_path.startswith(prefix):
            matches.append(relative_path)

    return matches


def add_factory_verb(subparsers):
    parser = subparsers.add_parser(
        "factory",
        help="Run a YASMIN state machine from an XML file",
        description="Run a YASMIN state machine from an XML file",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        help="Path to the XML state machine file",
    )
    xml_arg.completer = _xml_file_completer

    parser.add_argument(
        "--disable_viewer_pub",
        action="store_true",
        help="Disable FSM viewer publisher",
    )
    parser.add_argument(
        "--py",
        action="store_true",
        help="Use the Python factory node instead of the C++ factory node",
    )

    parser.set_defaults(main=_main_factory)


def _main_factory(args):
    executable = "yasmin_factory_node.py" if args.py else "yasmin_factory_node"

    command = [
        "ros2",
        "run",
        "yasmin_factory",
        executable,
        "--ros-args",
        "-p",
        f"state_machine_file:={args.state_machine_file}",
        "-p",
        f"enable_viewer_pub:={'false' if args.disable_viewer_pub else 'true'}",
    ]

    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1
