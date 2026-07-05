# Copyright (C) 2026 Miguel Ángel González Santamarta
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

from yasmin_editor.qt_compat import (
    QT_VERSION,
    QAction,
    Qt,
    QtCore,
    QtGui,
    QtTest,
    QtWidgets,
    exec_app,
    exec_dialog,
    exec_menu,
    mouse_event_global_pos,
    pyqtSignal,
)


class TestModuleExports:
    """Top-level symbols are present and have the expected types."""

    def test_qt_version_is_5_or_6(self) -> None:
        assert QT_VERSION in (5, 6)

    def test_qtcore_exported(self) -> None:
        assert QtCore is not None

    def test_qtgui_exported(self) -> None:
        assert QtGui is not None

    def test_qtwidgets_exported(self) -> None:
        assert QtWidgets is not None

    def test_qttest_exported(self) -> None:
        assert QtTest is not None

    def test_qt_enum_namespace(self) -> None:
        # Scoped-enum syntax must work on both PyQt5 >= 5.15 and PyQt6
        assert Qt.MouseButton.LeftButton is not None

    def test_pyqt_signal_exported(self) -> None:
        assert pyqtSignal is not None
        # Verify it is usable
        signal = pyqtSignal(str)
        assert signal is not None

    def test_qaction_exported(self) -> None:
        assert QAction is not None


class TestExecDialog:
    """exec_dialog dispatches to the right method."""

    def test_calls_exec_when_available(self) -> None:
        class _Mock:
            def exec(self) -> int:
                return 42

        assert exec_dialog(_Mock()) == 42

    def test_calls_exec_otherwise(self) -> None:
        class _Mock:
            def exec_(self) -> int:  # noqa: N802
                return 99

        assert exec_dialog(_Mock()) == 99

    def test_prefers_exec_over_exec_(self) -> None:
        class _Mock:
            def exec(self) -> int:
                return 1

            def exec_(self) -> int:  # noqa: N802
                return 2

        assert exec_dialog(_Mock()) == 1


class TestExecApp:
    """exec_app dispatches to the right method."""

    def test_calls_exec_when_available(self) -> None:
        class _Mock:
            def exec(self) -> int:
                return 10

        assert exec_app(_Mock()) == 10

    def test_calls_exec_otherwise(self) -> None:
        class _Mock:
            def exec_(self) -> int:  # noqa: N802
                return 20

        assert exec_app(_Mock()) == 20


class TestExecMenu:
    """exec_menu dispatches to the right method and passes the position."""

    def test_calls_exec_when_available(self) -> None:
        class _Mock:
            def exec(self, pos: object) -> object:
                return ("exec", pos)

        result = exec_menu(_Mock(), "pos_a")
        assert result == ("exec", "pos_a")

    def test_calls_exec_otherwise(self) -> None:
        class _Mock:
            def exec_(self, pos: object) -> object:  # noqa: N802
                return ("exec_", pos)

        result = exec_menu(_Mock(), "pos_b")
        assert result == ("exec_", "pos_b")

    def test_prefers_exec_over_exec_(self) -> None:
        class _Mock:
            def exec(self, pos: object) -> object:
                return "exec", pos

            def exec_(self, pos: object) -> object:  # noqa: N802
                return "exec_", pos

        result = exec_menu(_Mock(), "pos_c")
        assert result == ("exec", "pos_c")


class TestMouseEventGlobalPos:
    """mouse_event_global_pos returns the global position as a QPoint.

    The wrapper must handle both the PyQt5 ``globalPos()`` API and the
    PyQt6 ``globalPosition().toPoint()`` API.
    """

    def test_calls_global_position_when_available(self) -> None:
        class _Point:
            def toPoint(self) -> str:
                return "from_global_position"

        class _Mock:
            def globalPosition(self) -> _Point:
                return _Point()

        result = mouse_event_global_pos(_Mock())
        assert result == "from_global_position"

    def test_calls_global_pos_otherwise(self) -> None:
        class _Mock:
            def globalPos(self) -> str:
                return "from_global_pos"

        result = mouse_event_global_pos(_Mock())
        assert result == "from_global_pos"
