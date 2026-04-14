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

from types import SimpleNamespace

from yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin import EditorUiMixin
from yasmin_editor.model.concurrence import Concurrence
from yasmin_editor.model.outcome import Outcome
from yasmin_editor.model.state_machine import StateMachine


class DummyEditor(EditorUiMixin):
    def __init__(self, current_container_model) -> None:
        self.current_container_model = current_container_model
        self.created_connections = []

    def is_read_only_mode(self) -> bool:
        return False

    def _show_read_only_message(self) -> None:
        raise AssertionError("Read-only flow should not be used in this test")

    def create_connection(self, from_node, to_node, outcome: str) -> None:
        self.created_connections.append((from_node.name, to_node.name, outcome))


class FakePicker:
    last_init_kwargs = None
    outcomes_to_return = []

    def __init__(self, **kwargs) -> None:
        type(self).last_init_kwargs = kwargs

    def exec_(self) -> int:
        return 1

    def selected_outcomes(self):
        return list(type(self).outcomes_to_return)


class FakeStateMachinePicker(FakePicker):
    outcomes_to_return = ["done", "retry"]


class FakeFilteredStateMachinePicker(FakePicker):
    outcomes_to_return = ["retry", "failed"]


class FakeConcurrencePicker(FakePicker):
    outcomes_to_return = ["done", "failed"]


class FailingPicker:
    def __init__(self, **kwargs) -> None:
        raise AssertionError("Picker should not be opened")


class FakeFinalOutcomeNode:
    def __init__(self, name: str) -> None:
        self.name = name


def make_node(name: str, outcomes: list[str]):
    return SimpleNamespace(
        name=name,
        model=SimpleNamespace(outcomes=[Outcome(item) for item in outcomes]),
    )


def test_create_connection_from_drag_uses_picker_for_available_state_machine_outcomes(
    monkeypatch,
):
    editor = DummyEditor(StateMachine(name="sm"))
    from_node = make_node("worker", ["done", "retry", "failed"])
    to_node = SimpleNamespace(name="next_state")

    monkeypatch.setattr(
        "yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin.TransitionOutcomePickerDialog",
        FakeStateMachinePicker,
    )

    editor.create_connection_from_drag(from_node, to_node)

    assert FakeStateMachinePicker.last_init_kwargs["available_outcomes"] == [
        "done",
        "retry",
        "failed",
    ]
    assert editor.created_connections == [
        ("worker", "next_state", "done"),
        ("worker", "next_state", "retry"),
    ]


def test_create_connection_from_drag_allows_multi_select_for_concurrence(monkeypatch):
    editor = DummyEditor(Concurrence(name="cc"))
    from_node = make_node("worker", ["done", "failed"])
    to_node = FakeFinalOutcomeNode("finished")

    monkeypatch.setattr(
        "yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin.FinalOutcomeNode",
        FakeFinalOutcomeNode,
    )
    monkeypatch.setattr(
        "yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin.TransitionOutcomePickerDialog",
        FakeConcurrencePicker,
    )

    editor.create_connection_from_drag(from_node, to_node)

    assert FakeConcurrencePicker.last_init_kwargs["available_outcomes"] == [
        "done",
        "failed",
    ]
    assert editor.created_connections == [
        ("worker", "finished", "done"),
        ("worker", "finished", "failed"),
    ]


def test_create_connection_from_drag_filters_used_state_machine_outcomes(monkeypatch):
    editor = DummyEditor(StateMachine(name="sm"))
    editor.current_container_model.transitions = {
        "worker": [SimpleNamespace(source_outcome="done")]
    }
    from_node = make_node("worker", ["done", "retry", "failed"])
    to_node = SimpleNamespace(name="next_state")

    monkeypatch.setattr(
        "yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin.TransitionOutcomePickerDialog",
        FakeFilteredStateMachinePicker,
    )

    editor.create_connection_from_drag(from_node, to_node)

    assert FakeFilteredStateMachinePicker.last_init_kwargs["available_outcomes"] == [
        "retry",
        "failed",
    ]
    assert editor.created_connections == [
        ("worker", "next_state", "retry"),
        ("worker", "next_state", "failed"),
    ]


def test_create_connection_from_drag_skips_picker_when_only_one_outcome_is_available(
    monkeypatch,
):
    editor = DummyEditor(StateMachine(name="sm"))
    editor.current_container_model.transitions = {
        "worker": [
            SimpleNamespace(source_outcome="done"),
            SimpleNamespace(source_outcome="retry"),
        ]
    }
    from_node = make_node("worker", ["done", "retry", "failed"])
    to_node = SimpleNamespace(name="next_state")

    monkeypatch.setattr(
        "yasmin_editor.editor_gui.editor_mixin.editor_ui_mixin.TransitionOutcomePickerDialog",
        FailingPicker,
    )

    editor.create_connection_from_drag(from_node, to_node)

    assert editor.created_connections == [("worker", "next_state", "failed")]
