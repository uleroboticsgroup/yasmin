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

"""Qt-free helpers for canvas navigation, read-only state, and XML references."""

from __future__ import annotations

import os
from typing import Iterable

from yasmin_editor.model.concurrence import Concurrence

_XML_PATH_ATTRIBUTES = (
    "file_path",
    "xml_file",
    "path",
    "full_path",
    "abs_path",
    "filepath",
    "file_name",
)


def external_xml_view_active(
    extern_xml: object | None,
    extern_xml_path_start_index: int | None,
    current_path_length: int,
) -> bool:
    """Return whether the editor is currently inside an external XML subtree."""
    return (
        extern_xml is not None
        and extern_xml_path_start_index is not None
        and current_path_length > extern_xml_path_start_index
    )


def is_read_only_mode(
    runtime_mode_enabled: bool,
    extern_xml: object | None,
    extern_xml_path_start_index: int | None,
    current_path_length: int,
) -> bool:
    """Return whether the canvas must behave as read-only."""
    return runtime_mode_enabled or external_xml_view_active(
        extern_xml,
        extern_xml_path_start_index,
        current_path_length,
    )


def breadcrumb_label(
    index: int,
    container_model: object,
    *,
    extern_xml: object | None,
    extern_xml_source_state: object | None,
    extern_xml_path_start_index: int | None,
) -> str:
    """Return the label shown for one breadcrumb button."""
    if index == 0:
        return "root"

    if (
        extern_xml is not None
        and extern_xml_source_state is not None
        and extern_xml_path_start_index is not None
        and index == extern_xml_path_start_index
        and container_model is extern_xml
    ):
        return str(getattr(extern_xml_source_state, "name", ""))

    return str(getattr(container_model, "name", container_model))


def iter_xml_file_path_candidates(*sources: object | None) -> list[str]:
    """Collect candidate XML file paths from plugin and model metadata."""
    candidates: list[str] = []
    for source in sources:
        if source is None:
            continue
        for attr in _XML_PATH_ATTRIBUTES:
            value = getattr(source, attr, None)
            if value:
                candidates.append(str(value))
    return candidates


def _package_xml_file_candidates(
    package_name: str | None,
    file_name: str | None,
    *,
    package_share_lookup,
    file_exists,
    walk,
) -> Iterable[str]:
    """Yield candidate XML paths resolved through the ROS package share dir."""
    if not package_name or not file_name or package_share_lookup is None:
        return []

    try:
        share_dir = package_share_lookup(str(package_name))
    except Exception:
        return []

    basename = os.path.basename(str(file_name))
    direct_candidate = os.path.join(share_dir, str(file_name))
    candidates = [direct_candidate]
    for root_dir, _dirs, files in walk(share_dir):
        if basename in files:
            candidates.append(os.path.join(root_dir, basename))
    return [candidate for candidate in candidates if file_exists(candidate)]


def _default_package_share_lookup(package_name: str) -> str:
    """Resolve one ROS package share directory using ament."""
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package_name)


def resolve_xml_state_file_path(
    plugin_info: object | None,
    state_model: object | None,
    *,
    file_exists=os.path.isfile,
    walk=os.walk,
    package_share_lookup=None,
) -> str | None:
    """Resolve the XML file path for an external XML state reference."""
    for candidate in iter_xml_file_path_candidates(plugin_info, state_model):
        if file_exists(candidate):
            return candidate

    package_name = None
    file_name = None
    for source in (plugin_info, state_model):
        if source is None:
            continue
        package_name = package_name or getattr(source, "package_name", None)
        file_name = file_name or getattr(source, "file_name", None)

    package_share_lookup = package_share_lookup or _default_package_share_lookup
    package_candidates = _package_xml_file_candidates(
        package_name,
        file_name,
        package_share_lookup=package_share_lookup,
        file_exists=file_exists,
        walk=walk,
    )
    return next(iter(package_candidates), None)


def state_has_available_outcomes(
    state_model: object | None,
    current_container_model: object,
) -> bool:
    """Return whether a state still exposes at least one connectable outcome."""
    if state_model is None:
        return False

    outcomes = list(getattr(state_model, "outcomes", []) or [])
    if not outcomes:
        return False

    if isinstance(current_container_model, Concurrence):
        return True

    transitions = getattr(current_container_model, "transitions", {}) or {}
    state_name = getattr(state_model, "name", None)
    used_outcomes = {
        transition.source_outcome for transition in transitions.get(state_name, [])
    }
    return any(
        getattr(outcome, "name", None) not in used_outcomes for outcome in outcomes
    )
