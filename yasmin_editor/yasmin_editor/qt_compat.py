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

__all__ = [
    "QT_VERSION",
    "QtCore",
    "QtGui",
    "QtWidgets",
    "QtTest",
    "Qt",
    "pyqtSignal",
    "QAction",
    "exec_dialog",
    "exec_app",
    "exec_menu",
    "mouse_event_global_pos",
]

QT_VERSION: int

try:
    from PyQt6 import QtCore  # noqa: F401
    from PyQt6 import QtGui  # noqa: F401
    from PyQt6 import QtWidgets  # noqa: F401
    from PyQt6 import QtTest  # noqa: F401

    QT_VERSION = 6
except ImportError:
    from PyQt5 import QtCore  # noqa: F401
    from PyQt5 import QtGui  # noqa: F401
    from PyQt5 import QtWidgets  # noqa: F401
    from PyQt5 import QtTest  # noqa: F401

    QT_VERSION = 5

# Scoped-enum namespace – the ``Qt.MouseButton.LeftButton`` syntax
# works on PyQt5 ≥ 5.15 *and* on PyQt6.
Qt = QtCore.Qt

pyqtSignal = QtCore.pyqtSignal

# QAction moved from QtWidgets (PyQt5) to QtGui (PyQt6)
try:
    QAction = QtGui.QAction
except AttributeError:
    QAction = QtWidgets.QAction


def _call_dialog_exec(dialog) -> int:
    """Call *exec* or *exec_* whichever the dialog object provides.

    The try/fallback pattern makes the wrapper work with real Qt dialogs
    (where PyQt5 exposes ``exec_`` and PyQt6 exposes ``exec``) as well as
    with test mock objects that may only implement one of the two names.
    """
    fn = getattr(dialog, "exec", None)
    if fn is not None:
        return fn()
    return dialog.exec_()


def exec_dialog(dialog) -> int:
    """Execute a modal ``QDialog``."""
    return _call_dialog_exec(dialog)


def exec_app(app) -> int:
    """Start the ``QApplication`` event loop."""
    return _call_dialog_exec(app)


def exec_menu(menu, pos):
    """Show a popup ``QMenu`` at *pos*."""
    fn = getattr(menu, "exec", None)
    if fn is not None:
        return fn(pos)
    return menu.exec_(pos)


def mouse_event_global_pos(event) -> QtCore.QPoint:
    """Return the global position of a mouse *event* as a ``QPoint``.

    PyQt6 deprecated ``globalPos()`` in favour of ``globalPosition()``
    (which returns a ``QPointF``), so we abstract the difference away.
    """
    fn = getattr(event, "globalPosition", None)
    if fn is not None:
        return fn().toPoint()
    return event.globalPos()
