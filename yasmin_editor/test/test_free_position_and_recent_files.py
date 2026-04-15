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

"""Tests for free-position search and recent-file persistence helpers."""

import json
from pathlib import Path

from yasmin_editor.editor_gui.free_position import (
    fallback_position,
    find_free_position,
    is_position_free,
    iter_candidate_positions,
)
from yasmin_editor.editor_gui.recent_files import (
    RecentFilesStore,
    normalize_recent_file_path,
    prune_recent_file_entries,
    update_recent_file_entries,
)


def test_iter_candidate_positions_starts_at_center_and_expands_in_rings():
    candidates = iter_candidate_positions(
        (0.0, 0.0),
        spacing_x=10.0,
        spacing_y=5.0,
        radius_limit=1,
    )

    assert candidates[0] == (0.0, 0.0)
    assert len(candidates) == 9
    assert (-10.0, -5.0) in candidates
    assert (10.0, 5.0) in candidates


def test_find_free_position_skips_occupied_slots_and_uses_fallback_when_needed():
    occupied = [(0.0, 0.0), (10.0, 0.0)]
    assert find_free_position(
        (0.0, 0.0),
        occupied,
        spacing_x=10.0,
        spacing_y=10.0,
        radius_limit=1,
    ) != (0.0, 0.0)

    saturated = [(x * 10.0, y * 10.0) for y in (-1, 0, 1) for x in (-1, 0, 1)]
    assert find_free_position(
        (0.0, 0.0),
        saturated,
        spacing_x=10.0,
        spacing_y=10.0,
        radius_limit=1,
    ) == fallback_position((0.0, 0.0), len(saturated), spacing_x=10.0, spacing_y=10.0)


def test_is_position_free_uses_spacing_thresholds():
    assert not is_position_free(
        (7.0, 7.0),
        [(0.0, 0.0)],
        spacing_x=10.0,
        spacing_y=10.0,
    )
    assert is_position_free(
        (9.0, 9.0),
        [(0.0, 0.0)],
        spacing_x=10.0,
        spacing_y=10.0,
    )


def test_recent_file_helpers_deduplicate_and_normalize_paths(tmp_path: Path):
    first = tmp_path / "first.xml"
    second = tmp_path / "second.xml"
    first.write_text("a", encoding="utf-8")
    second.write_text("b", encoding="utf-8")

    entries = prune_recent_file_entries(
        [str(first), str(second), str(first), "", f"{tmp_path}/./second.xml"]
    )

    assert entries == [
        normalize_recent_file_path(str(first)),
        normalize_recent_file_path(str(second)),
    ]

    updated = update_recent_file_entries(entries, str(second), max_entries=2)
    assert updated == [
        normalize_recent_file_path(str(second)),
        normalize_recent_file_path(str(first)),
    ]


def test_recent_files_store_load_save_add_remove_and_clear(tmp_path: Path):
    store_path = tmp_path / "config" / "recent.json"
    store = RecentFilesStore(store_path, max_entries=3)

    first = tmp_path / "a.xml"
    second = tmp_path / "b.xml"
    first.write_text("a", encoding="utf-8")
    second.write_text("b", encoding="utf-8")

    assert store.load_entries() == []

    saved = store.save_entries([str(first), str(second), str(first)])
    assert saved == [
        normalize_recent_file_path(str(first)),
        normalize_recent_file_path(str(second)),
    ]
    assert json.loads(store_path.read_text(encoding="utf-8")) == saved

    added = store.add_file(str(second), current_entries=[str(first)])
    assert added[0] == normalize_recent_file_path(str(second))

    removed = store.remove_file(str(first), current_entries=added)
    assert removed == [normalize_recent_file_path(str(second))]

    store.clear()
    assert not store_path.exists()
