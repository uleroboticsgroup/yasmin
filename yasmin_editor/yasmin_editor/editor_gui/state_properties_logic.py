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

"""Qt-free helpers for the state-properties dialog.

The dialog is one of the editor's most form-heavy widgets and historically
mixed user-interface code with formatting and normalization rules. These
helpers keep the display-text, plugin-list, outcome, and remapping rules in a
single testable module so the dialog itself can stay focused on Qt widgets.
"""

from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from yasmin_plugins_manager.plugin_info import PluginInfo
except ModuleNotFoundError:  # pragma: no cover - exercised in headless tests

    class PluginInfo:  # type: ignore[override]
        """Fallback shim used when the plugin manager is unavailable."""

        @staticmethod
        def _normalize_cpp_metadata_type(type_name: str) -> str:
            return type_name


def normalize_display_type(type_name: str) -> str:
    """Return a stable user-facing type string.

    The plugin manager exposes a C++ metadata normalization helper. The dialog
    should use it when available but remain resilient when metadata is partial
    or malformed.
    """

    normalized_type = str(type_name or "").strip()
    if not normalized_type:
        return ""

    try:
        return PluginInfo._normalize_cpp_metadata_type(normalized_type)
    except Exception:
        return normalized_type


def _display_metadata_type(metadata: Dict[str, str]) -> str:
    """Return the user-facing type for one metadata row."""

    explicit_type = str(metadata.get("type", "") or "").strip()
    if explicit_type:
        return normalize_display_type(explicit_type)

    fallback_type = str(
        metadata.get("default_value_type", metadata.get("default_type", "")) or ""
    ).strip()
    return fallback_type


def _metadata_default_value_text(metadata: Dict[str, str]) -> str:
    """Return the normalized default-value text for one metadata row."""

    if metadata.get("has_default") or metadata.get("default_value") not in (None, ""):
        return str(metadata.get("default_value", "") or "").strip()
    return ""


def format_key_line(key_info: Dict[str, str], is_input: bool) -> str:
    """Return one formatted input/output key line for the description panel."""

    key_name = str(key_info.get("name", "")).strip()
    key_desc = str(key_info.get("description", "")).strip()
    key_type = _display_metadata_type(key_info)

    line = key_name if key_name else "(unnamed)"
    if key_desc:
        line += f": {key_desc}"

    default_value = _metadata_default_value_text(key_info) if is_input else ""
    if default_value:
        line += f" Default: {default_value}"

    if key_type:
        line += f" ({key_type})"

    return line


def format_parameter_line(parameter_info: Dict[str, str]) -> str:
    """Return one formatted parameter line for the description panel."""

    param_name = str(parameter_info.get("name", "")).strip()
    param_desc = str(parameter_info.get("description", "")).strip()
    param_type = _display_metadata_type(parameter_info)

    line = param_name if param_name else "(unnamed)"
    if param_desc:
        line += f": {param_desc}"
    default_value = _metadata_default_value_text(parameter_info)
    if default_value:
        line += f" Default: {default_value}"
    if param_type:
        line += f" ({param_type})"
    return line


def build_description_text(
    plugin_info: Optional[PluginInfo],
    fallback_description: str = "",
    fallback_outcomes: Optional[Sequence[str]] = None,
    fallback_parameters: Optional[Sequence[Dict[str, str]]] = None,
    fallback_input_keys: Optional[Sequence[Dict[str, str]]] = None,
    fallback_output_keys: Optional[Sequence[Dict[str, str]]] = None,
) -> str:
    """Build the read-only description text shown in the dialog.

    The dialog can either describe a plugin-backed state or a container/fallback
    configuration without a plugin instance.
    """

    if plugin_info:
        base_description = str(getattr(plugin_info, "description", "") or "").strip()
        input_keys = list(getattr(plugin_info, "input_keys", []) or [])
        output_keys = list(getattr(plugin_info, "output_keys", []) or [])
        parameters = list(getattr(plugin_info, "parameters", []) or [])
        outcomes = list(getattr(plugin_info, "outcomes", []) or [])
        outcome_descriptions = dict(
            getattr(plugin_info, "outcome_descriptions", {}) or {}
        )
    else:
        base_description = fallback_description.strip()
        input_keys = list(fallback_input_keys or [])
        output_keys = list(fallback_output_keys or [])
        parameters = list(fallback_parameters or [])
        outcomes = list(fallback_outcomes or [])
        outcome_descriptions = {}

    sections: List[str] = []
    if base_description:
        sections.append(base_description)

    if outcomes:
        if sections:
            sections.append("")
        sections.append("Outcomes:")
        for outcome in outcomes:
            line = f" - {outcome}"
            desc = outcome_descriptions.get(outcome)
            if desc:
                line += f": {desc}"
            sections.append(line)

    if parameters:
        if sections:
            sections.append("")
        lines = ["Parameters:"]
        for parameter_info in parameters:
            lines.append(" - " + format_parameter_line(parameter_info))
        sections.append("\n".join(lines))

    if input_keys:
        if sections:
            sections.append("")
        lines = ["Input Keys:"]
        for key_info in input_keys:
            lines.append(" - " + format_key_line(key_info, True))
        sections.append("\n".join(lines))

    if output_keys:
        if sections:
            sections.append("")
        lines = ["Output Keys:"]
        for key_info in output_keys:
            lines.append(" - " + format_key_line(key_info, False))
        sections.append("\n".join(lines))

    return "\n".join(sections).strip()


def plugin_display_name(plugin: PluginInfo) -> str:
    """Return the dialog label for one plugin entry."""

    plugin_type = getattr(plugin, "plugin_type", "")
    if plugin_type == "python":
        return f"{plugin.module}.{plugin.class_name}"
    if plugin_type == "cpp":
        return str(plugin.class_name)
    if plugin_type == "xml":
        filename = os.path.basename(plugin.file_name)
        if plugin.package_name:
            return f"{plugin.package_name}::{filename}"
        return filename
    return str(getattr(plugin, "class_name", "") or getattr(plugin, "name", "") or "")


def plugin_entries_for_type(
    available_plugins: Sequence[PluginInfo], plugin_type: str
) -> List[Tuple[str, PluginInfo]]:
    """Return display-name/plugin tuples for one type selector value."""

    entries: List[Tuple[str, PluginInfo]] = []
    for plugin in available_plugins:
        if getattr(plugin, "plugin_type", "") != plugin_type:
            continue
        entries.append((plugin_display_name(plugin), plugin))
    return entries


def declared_state_parameters(
    plugin_info: Optional[PluginInfo],
    fallback_parameters: Optional[Sequence[Dict[str, str]]] = None,
) -> List[Dict[str, str]]:
    """Return the parameter declarations visible to the dialog.

    Plugin-backed states expose declarations through plugin metadata. Container
    dialogs and fallback cases reuse the explicit fallback rows instead.
    """

    if plugin_info is not None:
        return list(getattr(plugin_info, "parameters", []) or [])
    return list(fallback_parameters or [])


def normalize_parameter_overwrite_row(parameter_data: Dict[str, str]) -> Dict[str, str]:
    """Return one stable parameter-overwrite row dictionary."""

    return {
        "name": str(parameter_data.get("name", "") or "").strip(),
        "child_parameter": str(parameter_data.get("child_parameter", "") or "").strip(),
        "description": str(parameter_data.get("description", "") or "").strip(),
        "default_type": str(parameter_data.get("default_type", "") or "").strip(),
        "default_value": str(parameter_data.get("default_value", "") or "").strip(),
    }


def collect_parameter_overwrites(
    rows: Iterable[Dict[str, str]],
) -> List[Dict[str, str]]:
    """Return normalized overwrite rows with incomplete entries removed."""

    overwrites: List[Dict[str, str]] = []
    for row in rows:
        normalized = normalize_parameter_overwrite_row(row)
        if normalized["name"] and normalized["child_parameter"]:
            overwrites.append(normalized)
    return overwrites


def collect_remappings(
    remapping_rows: Iterable[Tuple[str, str]],
) -> Dict[str, str]:
    """Return the normalized remapping map from table-like row values."""

    remappings: Dict[str, str] = {}
    for old_key, new_key in remapping_rows:
        normalized_old = str(old_key or "").strip()
        normalized_new = str(new_key or "").strip()
        if normalized_old and normalized_new:
            remappings[normalized_old] = normalized_new
    return remappings


def resolve_outcomes(
    plugin_info: Optional[PluginInfo], fallback_outcomes: Sequence[str]
) -> List[str]:
    """Return the outcome list used by the dialog result payload."""

    if plugin_info is not None and hasattr(plugin_info, "outcomes"):
        return list(getattr(plugin_info, "outcomes", []) or [])
    return list(fallback_outcomes)
