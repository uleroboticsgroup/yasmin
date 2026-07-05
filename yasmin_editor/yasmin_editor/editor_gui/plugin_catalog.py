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

from yasmin_editor.dataclass_compat import dataclass
from typing import Iterator, Sequence


@dataclass(frozen=True, slots=True)
class PluginListEntry:
    """A single sidebar entry for one plugin source item."""

    list_name: str
    display_name: str
    payload: object


def build_python_plugin_display_name(plugin: object) -> str:
    """Return the stable sidebar label for a Python state plugin."""

    return f"{plugin.module}.{plugin.class_name}"


def build_cpp_plugin_display_name(plugin: object) -> str:
    """Return the stable sidebar label for a C++ state plugin."""

    return plugin.class_name


def build_xml_plugin_display_name(xml_plugin: object) -> str:
    """Return the stable sidebar label for an XML state-machine file."""

    return f"{xml_plugin.package_name}/{xml_plugin.file_name}"


def iter_plugin_list_entries(plugin_manager: object) -> Iterator[PluginListEntry]:
    """Yield sidebar entries in the same order as the original editor.

    The order matters because the left sidebar is a high-frequency discovery
    surface. Keeping it deterministic also makes snapshot-like tests easier.
    """

    for plugin in plugin_manager.python_plugins:
        yield PluginListEntry(
            list_name="python_list",
            display_name=build_python_plugin_display_name(plugin),
            payload=plugin,
        )

    for plugin in plugin_manager.cpp_plugins:
        yield PluginListEntry(
            list_name="cpp_list",
            display_name=build_cpp_plugin_display_name(plugin),
            payload=plugin,
        )

    for xml_plugin in plugin_manager.xml_files:
        yield PluginListEntry(
            list_name="xml_list",
            display_name=build_xml_plugin_display_name(xml_plugin),
            payload=xml_plugin,
        )


def matches_plugin_filter(display_name: str, filter_text: str) -> bool:
    """Return whether one sidebar entry matches the current filter text.

    The sidebar filtering stays case-insensitive and treats an empty filter as a
    match for every entry. Keeping this rule outside Qt makes it easier to test
    and reuse anywhere the project needs the same list filtering semantics.
    """

    return filter_text.lower() in display_name.lower()


def list_widget_targets() -> Sequence[str]:
    """Return the canonical sidebar widget attribute names."""

    return ("python_list", "cpp_list", "xml_list")
