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


from pathlib import Path
import xml.etree.ElementTree as ET

from yasmin_editor.plugins_manager.plugin_manager import PluginManager

IGNORE_XML_FILES = {"package.xml", "plugins.xml"}


def plugin_id(plugin) -> str:
    if plugin.plugin_type == "python":
        return f"{plugin.module}.{plugin.class_name}"
    if plugin.plugin_type == "cpp":
        return plugin.class_name or ""
    if plugin.plugin_type == "xml":
        if plugin.package_name:
            return f"{plugin.package_name}/{plugin.file_name}"
        return plugin.file_name or ""
    return ""


def load_plugins(include_xml: bool = True):
    manager = PluginManager()
    manager.load_all_plugins(True)

    plugins = []
    plugins.extend(manager.cpp_plugins)
    plugins.extend(manager.python_plugins)

    if include_xml:
        plugins.extend(manager.xml_files)

    return sorted(plugins, key=plugin_id)


def find_plugin(plugin_name: str, include_xml: bool = True):
    for plugin in load_plugins(include_xml=include_xml):
        if plugin_id(plugin) == plugin_name:
            return plugin
    return None


def filter_plugins(plugins, plugin_type: str = "all", search: str | None = None):
    filtered = []
    lowered_search = search.lower() if search else None

    for plugin in plugins:
        if plugin_type != "all" and plugin.plugin_type != plugin_type:
            continue

        if lowered_search:
            haystack = [
                plugin_id(plugin),
                plugin.description or "",
                " ".join(plugin.outcomes),
                " ".join(key.get("name", "") for key in plugin.input_keys),
                " ".join(key.get("name", "") for key in plugin.output_keys),
            ]

            if not any(lowered_search in entry.lower() for entry in haystack):
                continue

        filtered.append(plugin)

    return sorted(filtered, key=plugin_id)


def plugin_completer(prefix, parsed_args, **kwargs):
    plugin_type = getattr(parsed_args, "type", "all")
    search = getattr(parsed_args, "search", None)

    matches: list[str] = []
    for plugin in filter_plugins(load_plugins(include_xml=True), plugin_type, search):
        current_plugin_id = plugin_id(plugin)
        if current_plugin_id.startswith(prefix):
            matches.append(current_plugin_id)

    return matches


def test_plugin_completer(prefix, parsed_args, **kwargs):
    matches: list[str] = []

    for plugin in load_plugins(include_xml=False):
        current_plugin_id = plugin_id(plugin)
        if current_plugin_id.startswith(prefix):
            matches.append(current_plugin_id)

    return matches


def input_completer(prefix, parsed_args, **kwargs):
    selected_plugin_id = getattr(parsed_args, "plugin_id", None)
    if not selected_plugin_id:
        return []

    plugin = find_plugin(selected_plugin_id, include_xml=False)
    if plugin is None:
        return []

    already_used = set()
    raw_inputs = getattr(parsed_args, "input", None) or []

    for entry in raw_inputs:
        if "=" not in entry:
            continue
        key, _ = entry.split("=", 1)
        already_used.add(key.strip())

    matches: list[str] = []
    for key in plugin.input_keys:
        name = key.get("name", "")
        if not name or name in already_used:
            continue

        suggestion = f"{name}="
        if suggestion.startswith(prefix):
            matches.append(suggestion)

    return matches


def strip_namespace(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def is_state_machine_xml(path: Path) -> bool:
    try:
        context = ET.iterparse(path, events=("start",))
        _, root = next(context)
        return strip_namespace(root.tag) == "StateMachine"
    except (ET.ParseError, OSError, StopIteration):
        return False


def xml_file_completer(prefix, parsed_args, **kwargs):
    current_dir = Path.cwd()
    matches: list[str] = []

    for path in sorted(current_dir.rglob("*.xml")):
        if path.name in IGNORE_XML_FILES:
            continue

        if not is_state_machine_xml(path):
            continue

        relative_path = path.relative_to(current_dir).as_posix()
        if relative_path.startswith(prefix):
            matches.append(relative_path)

    return matches
