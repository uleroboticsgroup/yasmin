from __future__ import annotations

import subprocess

from yasmin_cli.completer import xml_file_completer


def run_factory_node(
    state_machine_file: str,
    disable_viewer_pub: bool = False,
    use_python: bool = False,
) -> int:
    executable = "yasmin_factory_node.py" if use_python else "yasmin_factory_node"

    command = [
        "ros2",
        "run",
        "yasmin_factory",
        executable,
        "--ros-args",
        "-p",
        f"state_machine_file:={state_machine_file}",
        "-p",
        f"enable_viewer_pub:={'false' if disable_viewer_pub else 'true'}",
    ]

    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


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
    xml_arg.completer = xml_file_completer

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
    return run_factory_node(
        state_machine_file=args.state_machine_file,
        disable_viewer_pub=args.disable_viewer_pub,
        use_python=args.py,
    )
