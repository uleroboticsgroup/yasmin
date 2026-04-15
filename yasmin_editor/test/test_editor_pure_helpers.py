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

"""Tests for pure editor helpers that do not require a Qt event loop.

The covered helpers define clipboard rules, container metadata, document state,
startup argument parsing, plugin catalog formatting, canvas navigation, and
window geometry decisions.
"""

from pathlib import Path
from types import SimpleNamespace

from yasmin_editor.app import (
    build_qt_argv,
    parse_cli_args,
    startup_error_message,
    startup_open_message,
    viewport_is_ready,
)
from yasmin_editor.editor_gui.canvas_logic import (
    breadcrumb_label,
    external_xml_view_active,
    is_read_only_mode,
    iter_xml_file_path_candidates,
    resolve_xml_state_file_path,
    state_has_available_outcomes,
)
from yasmin_editor.editor_gui.clipboard_logic import (
    COPY_ACTION,
    MOVE_ACTION,
    PASTE_ACTION,
    clipboard_kind_label,
    clipboard_operation_status,
    cross_container_link_losses,
    cross_container_paste_warning,
    replacement_clipboard_message,
)
from yasmin_editor.editor_gui.clipboard_model import (
    create_clipboard_container,
    get_container_kind,
    is_container_empty,
)
from yasmin_editor.editor_gui.container_metadata_logic import (
    build_container_metadata_view,
    has_container_name_conflict,
    normalize_container_name,
)
from yasmin_editor.editor_gui.document_state import (
    EditorDirtyTracker,
    build_window_title,
    document_display_name,
)
from yasmin_editor.editor_gui.editor_action_state import (
    build_editor_action_enabled_map,
    toolbar_menu_enabled,
)
from yasmin_editor.editor_gui.plugin_catalog import (
    build_cpp_plugin_display_name,
    build_python_plugin_display_name,
    build_xml_plugin_display_name,
    iter_plugin_list_entries,
    list_widget_targets,
    matches_plugin_filter,
)
from yasmin_editor.editor_gui.selection_names import increment_name
from yasmin_editor.editor_gui.window_sizing import (
    WindowRect,
    build_initial_window_rect,
    choose_preferred_screen_rect,
    rect_contains_point,
)
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state import State
from yasmin_editor.model.state_machine import StateMachine
from yasmin_editor.model.text_block import TextBlock
from yasmin_editor.model.transition import Transition


class FakeViewport:
    """Tiny viewport stub used by the startup helper tests."""

    def __init__(self, width: int, height: int) -> None:
        self._width = width
        self._height = height

    def width(self) -> int:
        return self._width

    def height(self) -> int:
        return self._height


def make_leaf(name: str, outcomes: list[str]) -> State:
    """Create a leaf state with the given outcomes."""

    return State(name=name, outcomes=[Outcome(item) for item in outcomes])


def test_app_helpers_preserve_qt_arguments_and_format_startup_messages():
    """CLI parsing should keep Qt flags untouched and keep startup text stable."""

    args, unknown = parse_cli_args(["--xml-file", "demo.xml", "--style", "Fusion"])
    assert args.xml_file == "demo.xml"
    assert unknown == ["--style", "Fusion"]
    assert build_qt_argv("yasmin_editor", unknown) == [
        "yasmin_editor",
        "--style",
        "Fusion",
    ]
    assert viewport_is_ready(FakeViewport(20, 20)) is True
    assert viewport_is_ready(FakeViewport(8, 20)) is False
    assert startup_open_message("demo.xml") == "Opened: demo.xml"
    assert (
        startup_error_message(RuntimeError("broken"))
        == "Failed to open startup XML file: broken"
    )


def test_clipboard_helpers_report_container_switch_losses_and_status_messages():
    """Clipboard warnings should explain only the links that are actually lost."""

    assert clipboard_kind_label("concurrence") == "Concurrence"
    assert clipboard_kind_label("state_machine") == "State Machine"
    assert "state machine shelf" in replacement_clipboard_message("state_machine")
    assert cross_container_link_losses(
        "state_machine",
        "concurrence",
        has_transitions=True,
        has_outcome_rules=False,
    ) == ["state-machine transitions"]
    assert cross_container_link_losses(
        "concurrence",
        "state_machine",
        has_transitions=False,
        has_outcome_rules=True,
    ) == ["concurrence outcome rules"]
    assert (
        cross_container_link_losses(
            "state_machine",
            "state_machine",
            has_transitions=True,
            has_outcome_rules=True,
        )
        == []
    )
    assert "cannot be preserved" in cross_container_paste_warning(
        "concurrence",
        "state_machine",
        has_transitions=False,
        has_outcome_rules=True,
    )
    assert (
        cross_container_paste_warning(
            "state_machine",
            "state_machine",
            has_transitions=True,
            has_outcome_rules=True,
        )
        is None
    )
    assert (
        clipboard_operation_status(COPY_ACTION, performed=True)
        == "Prepared copied selection"
    )
    assert (
        clipboard_operation_status(PASTE_ACTION, performed=True)
        == "Placed stored selection"
    )
    assert (
        clipboard_operation_status(MOVE_ACTION, performed=True)
        == "Moved stored selection"
    )
    assert clipboard_operation_status("unknown", performed=True) is None
    assert clipboard_operation_status(COPY_ACTION, performed=False) is None


def test_clipboard_container_helpers_create_kind_specific_empty_models():
    """Shelf containers should use the right model type and emptiness rules."""

    state_machine = create_clipboard_container("state_machine")
    concurrence = create_clipboard_container("concurrence")

    assert isinstance(state_machine, StateMachine)
    assert isinstance(concurrence, Concurrence)
    assert get_container_kind(state_machine) == "state_machine"
    assert get_container_kind(concurrence) == "concurrence"
    assert is_container_empty(state_machine) is True
    assert is_container_empty(concurrence) is True

    state_machine.add_state(make_leaf("worker", ["done"]))
    assert is_container_empty(state_machine) is False
    state_machine.remove_state("worker")
    state_machine.add_transition(
        "worker", Transition(source_outcome="done", target="next")
    )
    assert is_container_empty(state_machine) is False

    concurrence.set_outcome_rule("finished", "worker", "done")
    assert is_container_empty(concurrence) is False


def test_container_metadata_and_document_state_helpers_track_ui_state():
    """Metadata views, dirty tracking, and title text should stay deterministic."""

    sm = StateMachine(name="root", start_state="worker")
    sm.add_state(make_leaf("worker", ["done"]))
    cc = Concurrence(
        name="parallel", default_outcome="finished", outcomes=[Outcome("finished")]
    )

    sm_view = build_container_metadata_view(sm)
    cc_view = build_container_metadata_view(cc)
    assert sm_view.selector_items == ["worker"]
    assert sm_view.current_selector_value == "worker"
    assert cc_view.selector_items == ["finished"]
    assert cc_view.current_selector_value == "finished"
    assert normalize_container_name("  demo  ") == "demo"
    assert (
        has_container_name_conflict(
            "existing",
            current_name="root",
            sibling_state_names=["existing"],
            sibling_outcome_names=["done"],
        )
        is True
    )
    assert (
        has_container_name_conflict(
            "root",
            current_name="root",
            sibling_state_names=["existing"],
            sibling_outcome_names=["done"],
        )
        is False
    )

    tracker = EditorDirtyTracker()
    assert tracker.is_dirty(sm) is False
    tracker.reset(sm)
    assert tracker.is_dirty(sm) is False
    sm.description = "changed"
    assert tracker.is_dirty(sm) is True
    assert document_display_name(None) == "Untitled"
    assert document_display_name("/tmp/demo.xml") == "demo.xml"
    assert (
        build_window_title("/tmp/demo.xml", is_dirty=True) == "* demo.xml - YASMIN Editor"
    )


def test_editor_action_and_plugin_catalog_helpers_expose_stable_rules():
    """Action enablement and sidebar plugin labels should stay easy to predict."""

    enabled_map = build_editor_action_enabled_map(
        read_only_mode=False,
        has_selection=True,
        has_state_selection=False,
        clipboard_has_content=False,
    )
    assert enabled_map["add_state_action"] is True
    assert enabled_map["delete_action"] is True
    assert enabled_map["extract_selection_action"] is False
    assert (
        toolbar_menu_enabled(("delete_action", "extract_selection_action"), enabled_map)
        is True
    )
    assert toolbar_menu_enabled(("extract_selection_action",), enabled_map) is False

    python_plugin = SimpleNamespace(module="demo_pkg.alpha", class_name="AlphaState")
    cpp_plugin = SimpleNamespace(class_name="CppState")
    xml_plugin = SimpleNamespace(package_name="demo_pkg", file_name="graph.xml")
    manager = SimpleNamespace(
        python_plugins=[python_plugin],
        cpp_plugins=[cpp_plugin],
        xml_files=[xml_plugin],
    )

    assert build_python_plugin_display_name(python_plugin) == "demo_pkg.alpha.AlphaState"
    assert build_cpp_plugin_display_name(cpp_plugin) == "CppState"
    assert build_xml_plugin_display_name(xml_plugin) == "demo_pkg/graph.xml"
    entries = list(iter_plugin_list_entries(manager))
    assert [(entry.list_name, entry.display_name) for entry in entries] == [
        ("python_list", "demo_pkg.alpha.AlphaState"),
        ("cpp_list", "CppState"),
        ("xml_list", "demo_pkg/graph.xml"),
    ]
    assert matches_plugin_filter("demo_pkg.alpha.AlphaState", "alpha") is True
    assert matches_plugin_filter("demo_pkg.alpha.AlphaState", "BETA") is False
    assert list_widget_targets() == ("python_list", "cpp_list", "xml_list")
    assert increment_name("worker", {"worker", "worker2"}) == "worker3"
    assert increment_name("", set()) == "state"


def test_canvas_helpers_resolve_external_xml_paths_and_available_outcomes(tmp_path: Path):
    """Canvas navigation helpers should resolve labels, file paths, and transition gaps."""

    xml_file = tmp_path / "graphs" / "demo.xml"
    xml_file.parent.mkdir(parents=True)
    xml_file.write_text("<state-machine />", encoding="utf-8")

    assert external_xml_view_active(object(), 1, 2) is True
    assert external_xml_view_active(object(), 1, 1) is False
    assert is_read_only_mode(False, object(), 0, 2) is True
    assert is_read_only_mode(True, None, None, 0) is True
    assert (
        breadcrumb_label(
            0,
            object(),
            extern_xml=None,
            extern_xml_source_state=None,
            extern_xml_path_start_index=None,
        )
        == "root"
    )

    external_model = SimpleNamespace(name="external_container")
    external_source = SimpleNamespace(name="xml_state")
    assert (
        breadcrumb_label(
            2,
            external_model,
            extern_xml=external_model,
            extern_xml_source_state=external_source,
            extern_xml_path_start_index=2,
        )
        == "xml_state"
    )

    candidates = iter_xml_file_path_candidates(
        SimpleNamespace(file_path=str(xml_file)),
        SimpleNamespace(file_name="demo.xml", abs_path=str(xml_file)),
    )
    assert candidates[0] == str(xml_file)
    assert str(xml_file) in candidates
    assert "demo.xml" in candidates

    resolved_direct = resolve_xml_state_file_path(
        SimpleNamespace(file_path=str(xml_file)),
        None,
        file_exists=lambda candidate: candidate == str(xml_file),
        walk=lambda _root: [],
        package_share_lookup=lambda _package: str(tmp_path),
    )
    assert resolved_direct == str(xml_file)

    resolved_package = resolve_xml_state_file_path(
        SimpleNamespace(package_name="demo_pkg", file_name="demo.xml"),
        None,
        file_exists=lambda candidate: candidate == str(xml_file),
        walk=lambda _root: [(str(xml_file.parent), [], [xml_file.name])],
        package_share_lookup=lambda _package: str(tmp_path),
    )
    assert resolved_package == str(xml_file)

    state_machine = StateMachine(name="root")
    worker = make_leaf("worker", ["done", "retry"])
    state_machine.add_state(worker)
    state_machine.add_transition(
        "worker", Transition(source_outcome="done", target="next")
    )
    assert state_has_available_outcomes(worker, state_machine) is True
    state_machine.add_transition(
        "worker", Transition(source_outcome="retry", target="next")
    )
    assert state_has_available_outcomes(worker, state_machine) is False
    assert state_has_available_outcomes(worker, Concurrence(name="parallel")) is True
    assert state_has_available_outcomes(None, state_machine) is False


def test_window_geometry_helpers_select_the_expected_screen_and_center_window():
    """Initial window geometry should honor cursor location and available space."""

    left = WindowRect(x=0, y=0, width=1920, height=1080)
    right = WindowRect(x=1920, y=0, width=2560, height=1440)

    assert rect_contains_point(left, x=100, y=100) is True
    assert rect_contains_point(left, x=1920, y=100) is False
    assert (
        choose_preferred_screen_rect([left, right], cursor_x=2000, cursor_y=10) == right
    )
    assert (
        choose_preferred_screen_rect(
            [left, right], cursor_x=None, cursor_y=None, fallback_index=5
        )
        == right
    )

    rect = build_initial_window_rect(
        0, 0, 800, 600, preferred_width=2000, preferred_height=2000
    )
    assert rect.width <= 800
    assert rect.height <= 600
    assert rect.x >= 0
    assert rect.y >= 0
    assert rect.x + rect.width <= 800
    assert rect.y + rect.height <= 600
