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

"""Helpers for storing and curating recently opened editor documents."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Iterable

DEFAULT_MAX_RECENT_FILES = 10


def default_recent_files_store_path() -> Path:
    """Return the default JSON file used for recent document persistence."""

    config_home = os.environ.get("XDG_CONFIG_HOME")
    if config_home:
        return Path(config_home) / "yasmin_editor" / "recent_files.json"
    return Path.home() / ".config" / "yasmin_editor" / "recent_files.json"


def normalize_recent_file_path(file_path: str) -> str:
    """Return one canonical form for a recent-document path."""

    return os.path.normpath(os.path.abspath(os.path.expanduser(file_path)))


def _deduplicate_paths(file_paths: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    ordered_paths: list[str] = []
    for path in file_paths:
        if not path:
            continue
        normalized = normalize_recent_file_path(path)
        if normalized in seen:
            continue
        seen.add(normalized)
        ordered_paths.append(normalized)
    return ordered_paths


def prune_recent_file_entries(
    file_paths: Iterable[str], *, existing_only: bool = False
) -> list[str]:
    """Return cleaned recent-file entries with optional existence filtering."""

    entries = _deduplicate_paths(file_paths)
    if not existing_only:
        return entries
    return [path for path in entries if os.path.exists(path)]


def update_recent_file_entries(
    file_paths: Iterable[str],
    file_path: str,
    *,
    max_entries: int = DEFAULT_MAX_RECENT_FILES,
) -> list[str]:
    """Move one path to the front of the recent-files list."""

    normalized_file_path = normalize_recent_file_path(file_path)
    entries = [normalized_file_path]
    entries.extend(prune_recent_file_entries(file_paths))
    return prune_recent_file_entries(entries)[:max_entries]


class RecentFilesStore:
    """Persist recent editor documents in a small JSON file."""

    def __init__(
        self,
        file_path: str | Path | None = None,
        *,
        max_entries: int = DEFAULT_MAX_RECENT_FILES,
    ) -> None:
        self.file_path = (
            Path(file_path)
            if file_path is not None
            else default_recent_files_store_path()
        )
        self.max_entries = max_entries

    def load_entries(self, *, existing_only: bool = False) -> list[str]:
        """Load recent-file entries from disk.

        Invalid or unreadable files degrade gracefully to an empty list so the
        editor startup path does not fail because of configuration issues.
        """

        try:
            raw_content = self.file_path.read_text(encoding="utf-8")
            payload = json.loads(raw_content)
        except (FileNotFoundError, OSError, json.JSONDecodeError, TypeError, ValueError):
            return []

        if not isinstance(payload, list):
            return []
        return prune_recent_file_entries(payload, existing_only=existing_only)[
            : self.max_entries
        ]

    def save_entries(self, file_paths: Iterable[str]) -> list[str]:
        """Write recent-file entries to disk and return the stored list."""

        entries = prune_recent_file_entries(file_paths)[: self.max_entries]
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text(json.dumps(entries, indent=2), encoding="utf-8")
        return entries

    def add_file(
        self, file_path: str, current_entries: Iterable[str] | None = None
    ) -> list[str]:
        """Add one file path to the persistent recent-files list."""

        base_entries = (
            list(current_entries) if current_entries is not None else self.load_entries()
        )
        entries = update_recent_file_entries(
            base_entries,
            file_path,
            max_entries=self.max_entries,
        )
        return self.save_entries(entries)

    def remove_file(
        self,
        file_path: str,
        current_entries: Iterable[str] | None = None,
    ) -> list[str]:
        """Remove one file path from the persistent recent-files list."""

        normalized_file_path = normalize_recent_file_path(file_path)
        base_entries = (
            list(current_entries) if current_entries is not None else self.load_entries()
        )
        entries = [
            path
            for path in prune_recent_file_entries(base_entries)
            if path != normalized_file_path
        ]
        return self.save_entries(entries)

    def clear(self) -> None:
        """Clear all persisted recent-file entries."""

        try:
            self.file_path.unlink()
        except FileNotFoundError:
            return
