#!/usr/bin/env python3

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

from __future__ import annotations

from pathlib import Path

from ament_index_python import get_package_share_path
from yasmin_factory import YasminFactory
from yasmin_plugins_manager import PluginManager

from yasmin_cli.completer import xml_file_completer


def _resolve_plugin_xml_files() -> list[tuple[str, Path]]:
    """
    Collect all XML state machine files known to the plugin manager.

    Returns
    -------
    list[tuple[str, Path]]
        Pairs of display label and absolute XML path.
    """
    plugin_manager = PluginManager()
    plugin_manager.load_all_plugins(hide_progress=True)

    xml_files: list[tuple[str, Path]] = []

    for plugin in plugin_manager.xml_files:
        if not plugin.package_name:
            continue

        package_share_path = Path(get_package_share_path(plugin.package_name))

        if plugin.relative_path:
            xml_path = (package_share_path / plugin.relative_path).resolve()
        elif plugin.file_name:
            xml_path = (package_share_path / plugin.file_name).resolve()
        else:
            continue

        label = f"{plugin.package_name}/{plugin.relative_path or plugin.file_name}"
        xml_files.append((label, xml_path))

    xml_files.sort(key=lambda item: item[0])
    return xml_files


def _validate_xml_file(xml_file: str, strict_mode: bool) -> tuple[bool, str]:
    """
    Create a state machine from XML using the factory and validate it.

    Parameters
    ----------
    xml_file : str
        Path to the XML state machine file.
    strict_mode : bool
        Whether strict validation should be enabled.

    Returns
    -------
    tuple[bool, str]
        Validation success flag and a message.
    """
    try:
        factory = YasminFactory()
        state_machine = factory.create_sm_from_file(xml_file)
        state_machine.validate(strict_mode=strict_mode)
        return True, "OK"
    except Exception as exc:
        return False, str(exc)


def _validate_single_file(state_machine_file: str, strict_mode: bool) -> int:
    """
    Validate one XML state machine file.

    Parameters
    ----------
    state_machine_file : str
        Path to the XML file.
    strict_mode : bool
        Whether strict validation should be enabled.

    Returns
    -------
    int
        CLI return code.
    """
    xml_path = Path(state_machine_file)

    if not xml_path.is_file():
        print(f"[FAIL] {state_machine_file}")
        print(f"       File does not exist: {state_machine_file}")
        return 1

    success, message = _validate_xml_file(str(xml_path), strict_mode)

    if success:
        print(f"[OK]   {xml_path}")
        return 0

    print(f"[FAIL] {xml_path}")
    print(f"       {message}")
    return 1


def _validate_all_plugin_xml_files(strict_mode: bool) -> int:
    """
    Validate all XML state machines discovered by the plugin manager.

    Parameters
    ----------
    strict_mode : bool
        Whether strict validation should be enabled.

    Returns
    -------
    int
        CLI return code.
    """
    xml_files = _resolve_plugin_xml_files()

    if not xml_files:
        print("No XML state machines found in the plugin manager.")
        return 0

    failed = 0

    for label, xml_path in xml_files:
        success, message = _validate_xml_file(str(xml_path), strict_mode)

        if success:
            print(f"[OK]   {label}")
        else:
            failed += 1
            print(f"[FAIL] {label}")
            print(f"       File: {xml_path}")
            print(f"       {message}")

    print("")
    print(
        f"Validated {len(xml_files)} XML state machine(s), "
        f"{len(xml_files) - failed} succeeded, {failed} failed."
    )

    return 0 if failed == 0 else 1


def add_validate_verb(subparsers):
    parser = subparsers.add_parser(
        "validate",
        help="Validate a YASMIN XML state machine",
        description="Create a YASMIN state machine from XML and call validate() on it",
    )

    xml_arg = parser.add_argument(
        "state_machine_file",
        nargs="?",
        default="",
        help="Path to the XML state machine file. If omitted, all XML files from the plugin manager are validated.",
    )
    xml_arg.completer = xml_file_completer

    parser.add_argument(
        "--no-strict",
        action="store_true",
        help="Disable strict mode when calling validate()",
    )

    parser.set_defaults(main=_main_validate)


def _main_validate(args):
    strict_mode = not args.no_strict

    if args.state_machine_file:
        return _validate_single_file(args.state_machine_file, strict_mode)

    return _validate_all_plugin_xml_files(strict_mode)
