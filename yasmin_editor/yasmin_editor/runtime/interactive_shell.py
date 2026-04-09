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

from __future__ import annotations

from dataclasses import dataclass
import builtins
from typing import Any, Callable, Optional

_MISSING = object()

from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import QAbstractItemView, QDialog, QVBoxLayout, QWidget

from yasmin_editor.editor_gui.theme import PALETTE
from yasmin_editor.editor_gui.theme.qt_style import (
    build_qtconsole_palette,
    build_qtconsole_stylesheet,
    build_qtconsole_syntax_style,
)

try:
    from qtconsole.inprocess import QtInProcessKernelManager
    from qtconsole.rich_jupyter_widget import RichJupyterWidget

    _QTCONSOLE_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:
    QtInProcessKernelManager = None
    RichJupyterWidget = None
    _QTCONSOLE_IMPORT_ERROR = exc


@dataclass
class BlackboardAccessError:
    key: str
    error: str

    def __repr__(self) -> str:
        return f"<BlackboardAccessError key={self.key!r} error={self.error!r}>"

    __str__ = __repr__


class SafeBlackboardProxy:
    def __init__(self, blackboard: Any) -> None:
        object.__setattr__(self, "_blackboard", blackboard)

    def _require_blackboard(self) -> Any:
        blackboard = object.__getattribute__(self, "_blackboard")
        if blackboard is None:
            raise RuntimeError("No active blackboard is available.")
        return blackboard

    def _safe_read(self, key: str) -> Any:
        try:
            return self._require_blackboard().get(key)
        except Exception as exc:
            return BlackboardAccessError(str(key), str(exc))

    def unwrap(self) -> Any:
        return self._require_blackboard()

    def get(self, key: str, default: Any = _MISSING) -> Any:
        value = self._safe_read(str(key))
        if isinstance(value, BlackboardAccessError):
            return default if default is not _MISSING else value
        return value

    def set(self, key: str, value: Any) -> None:
        self._require_blackboard().set(key, value)

    def remove(self, key: str) -> None:
        self._require_blackboard().remove(key)

    def contains(self, key: str) -> bool:
        try:
            return bool(self._require_blackboard().contains(key))
        except Exception:
            return False

    def size(self) -> int:
        try:
            return int(self._require_blackboard().size())
        except Exception:
            return 0

    def get_remappings(self) -> dict[str, str]:
        try:
            return dict(self._require_blackboard().get_remappings())
        except Exception:
            return {}

    def set_remappings(self, remappings: dict[str, str]) -> None:
        self._require_blackboard().set_remappings(remappings)

    def keys(self):
        return self._require_blackboard().keys()

    def items(self):
        return self._require_blackboard().items()

    def values(self):
        return self._require_blackboard().values()

    def __iter__(self):
        try:
            return iter(self._require_blackboard())
        except Exception:
            return iter(())

    def __contains__(self, key: str) -> bool:
        return self.contains(key)

    def __getitem__(self, key: str) -> Any:
        return self._safe_read(str(key))

    def __setitem__(self, key: str, value: Any) -> None:
        self.set(str(key), value)

    def __delitem__(self, key: str) -> None:
        self.remove(str(key))

    def __getattr__(self, name: str) -> Any:
        raise AttributeError(
            "Attribute-style blackboard access is not supported in the shell. "
            "Use bb['key'] or bb.get('key')."
        )

    def __setattr__(self, name: str, value: Any) -> None:
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        self.set(name, value)

    def __len__(self) -> int:
        return self.size()

    def __repr__(self) -> str:
        try:
            blackboard = self._require_blackboard()
            return f"<SafeBlackboardProxy size={blackboard.size()}>"
        except Exception:
            return "<SafeBlackboardProxy unavailable>"

    def __str__(self) -> str:
        try:
            return str(self._require_blackboard())
        except Exception as exc:
            return f"<SafeBlackboardProxy unavailable error={exc}>"

    def __dir__(self) -> list[str]:
        return sorted(
            {
                "contains",
                "get",
                "get_remappings",
                "remove",
                "set",
                "set_remappings",
                "size",
                "unwrap",
                "keys",
                "items",
                "values",
            }
        )


class _RuntimeShellCommand:
    def __init__(
        self,
        name: str,
        callback: Callable[[], Any],
        description: str,
    ) -> None:
        self._name = str(name)
        self._callback = callback
        self.__doc__ = description

    def _execute(self) -> str:
        result = self._callback()
        if result is None:
            return f"{self._name} executed"
        return str(result)

    def __call__(self) -> str:
        return self._execute()

    def __repr__(self) -> str:
        return self._execute()

    __str__ = __repr__


class _ShellDialog(QDialog):
    visibility_changed = pyqtSignal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent, Qt.Window)
        self._saved_geometry = None
        self._saved_show_mode = "normal"
        self.setWindowFlag(Qt.WindowMinimizeButtonHint, True)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, True)
        self.setWindowFlag(Qt.WindowCloseButtonHint, True)

    def preferred_show_mode(self) -> str:
        return str(self._saved_show_mode or "normal")

    def remember_window_state(self) -> None:
        self._saved_geometry = self.saveGeometry()
        if self.isFullScreen():
            self._saved_show_mode = "fullscreen"
        elif self.isMaximized():
            self._saved_show_mode = "maximized"
        else:
            self._saved_show_mode = "normal"

    def restore_window_state(self) -> None:
        if self._saved_geometry is not None:
            self.restoreGeometry(self._saved_geometry)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self.visibility_changed.emit(True)

    def hideEvent(self, event) -> None:
        self.remember_window_state()
        super().hideEvent(event)
        self.visibility_changed.emit(False)

    def closeEvent(self, event) -> None:
        self.remember_window_state()
        event.ignore()
        self.hide()


class InteractiveShellManager(QObject):
    visibility_changed = pyqtSignal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._dialog: Optional[_ShellDialog] = None
        self._widget: Optional[RichJupyterWidget] = None
        self._kernel_manager: Optional[QtInProcessKernelManager] = None
        self._kernel_client = None
        self._commands: dict[str, _RuntimeShellCommand] = {}
        self._banner = (
            "YASMIN interactive shell\n"
            "\n"
            "bb -> safe blackboard proxy\n"
            "sm -> root state machine\n"
            "current_state -> currently active state object or None after completion\n"
            "last_state -> previously active state object\n"
            "active_path -> tuple with the current runtime path\n"
            "last_transition -> latest observed runtime transition\n"
            "\n"
            "Debugger-style commands: next, step, cont, play, pause,\n"
            "cancel_state, cancel_sm, restart, where\n"
            "\n"
            "Use bb['key'] or bb.get('key').\n"
            "Attribute-style access is disabled to keep IPython introspection stable.\n"
            "Values that cannot be converted from C++ are returned as\n"
            "BlackboardAccessError instead of raising immediately.\n"
            "\n"
            "Recommendation: inspect the blackboard during runtime, but avoid\n"
            "modifying it while execution is running.\n"
        )

    @staticmethod
    def is_supported() -> bool:
        return QtInProcessKernelManager is not None and RichJupyterWidget is not None

    @staticmethod
    def unavailable_reason() -> str:
        if InteractiveShellManager.is_supported():
            return ""
        if _QTCONSOLE_IMPORT_ERROR is None:
            return "qtconsole is not available."
        return str(_QTCONSOLE_IMPORT_ERROR)

    def is_open(self) -> bool:
        return self._dialog is not None and self._dialog.isVisible()

    def open_shell(
        self,
        bb: Any,
        sm: Any,
        current_state: Any = None,
        last_state: Any = None,
        commands: Optional[dict[str, Callable[[], Any]]] = None,
        active_path: Any = None,
        last_transition: Any = None,
    ) -> None:
        self._ensure_shell()
        self._push_context(
            bb=bb,
            sm=sm,
            current_state=current_state,
            last_state=last_state,
            commands=commands,
            active_path=active_path,
            last_transition=last_transition,
        )

        if self._dialog is None:
            return

        self._apply_shell_theme()
        self._dialog.restore_window_state()
        show_mode = self._dialog.preferred_show_mode()
        if show_mode == "fullscreen":
            self._dialog.showFullScreen()
        elif show_mode == "maximized":
            self._dialog.showMaximized()
        else:
            self._dialog.show()
        self._dialog.raise_()
        self._dialog.activateWindow()

    def update_context(
        self,
        bb: Any,
        sm: Any,
        current_state: Any = None,
        last_state: Any = None,
        commands: Optional[dict[str, Callable[[], Any]]] = None,
        active_path: Any = None,
        last_transition: Any = None,
    ) -> None:
        self._ensure_shell()
        self._push_context(
            bb=bb,
            sm=sm,
            current_state=current_state,
            last_state=last_state,
            commands=commands,
            active_path=active_path,
            last_transition=last_transition,
        )
        self._apply_shell_theme()

    def close_shell(self) -> None:
        if self._dialog is not None:
            self._dialog.hide()

    def shutdown(self) -> None:
        if self._dialog is not None:
            self._dialog.hide()

        if self._kernel_client is not None:
            try:
                self._kernel_client.stop_channels()
            except Exception:
                pass
            self._kernel_client = None

        if self._kernel_manager is not None:
            try:
                self._kernel_manager.shutdown_kernel()
            except Exception:
                pass
            self._kernel_manager = None

        self._widget = None
        self._dialog = None

    def _register_command_magics(self) -> None:
        if self._kernel_manager is None:
            return

        shell = self._kernel_manager.kernel.shell
        if shell is None:
            return

        shell.automagic = True

        def _register_magic(name: str) -> None:
            def _magic(line: str = "") -> str:
                command = self._commands.get(name)
                return command() if command is not None else f"{name} unavailable"

            shell.register_magic_function(
                _magic,
                magic_kind="line",
                magic_name=name,
            )

        for command_name in [
            "next",
            "step",
            "cont",
            "play",
            "pause",
            "cancel_state",
            "cancel_sm",
            "restart",
            "where",
        ]:
            _register_magic(command_name)

    def _apply_widget_palette(self, widget: QWidget) -> None:
        shell_palette = build_qtconsole_palette(PALETTE)
        widget.setAutoFillBackground(True)
        widget.setPalette(shell_palette)

    def _apply_completion_theme(self, shell_palette, shell_stylesheet: str) -> None:
        if self._widget is None:
            return

        completion_widget = getattr(self._widget, "_completion_widget", None)
        if completion_widget is None:
            return

        try:
            completion_widget.setObjectName("qtconsoleCompletionWidget")
        except Exception:
            pass

        try:
            completion_widget.setAutoFillBackground(True)
            completion_widget.setPalette(shell_palette)
        except Exception:
            pass

        try:
            completion_widget.setStyleSheet(shell_stylesheet)
        except Exception:
            pass

        if isinstance(completion_widget, QAbstractItemView):
            try:
                completion_widget.viewport().setAutoFillBackground(True)
                completion_widget.viewport().setPalette(shell_palette)
            except Exception:
                pass
            try:
                completion_widget.viewport().setStyleSheet(shell_stylesheet)
            except Exception:
                pass

    def _apply_shell_theme(self) -> None:
        if self._widget is None:
            return

        shell_palette = build_qtconsole_palette(PALETTE)
        shell_stylesheet = build_qtconsole_stylesheet(PALETTE)
        syntax_style = build_qtconsole_syntax_style(PALETTE)

        self._widget.setAutoFillBackground(True)
        self._widget.setPalette(shell_palette)

        try:
            self._widget.style_sheet = shell_stylesheet
            if hasattr(self._widget, "_style_sheet_changed"):
                self._widget._style_sheet_changed()
        except Exception:
            pass

        try:
            self._widget.setStyleSheet(shell_stylesheet)
        except Exception:
            pass

        if syntax_style is not None:
            try:
                self._widget.syntax_style = syntax_style
                if hasattr(self._widget, "_syntax_style_changed"):
                    self._widget._syntax_style_changed()
            except Exception:
                pass

        for attr_name in ("_control", "_page_control"):
            control = getattr(self._widget, attr_name, None)
            if control is None:
                continue
            self._apply_widget_palette(control)
            try:
                control.setStyleSheet(shell_stylesheet)
            except Exception:
                pass
            try:
                viewport = control.viewport()
                viewport.setAutoFillBackground(True)
                viewport.setPalette(shell_palette)
                viewport.setStyleSheet(shell_stylesheet)
            except Exception:
                pass

        self._apply_completion_theme(shell_palette, shell_stylesheet)

        for child in self._widget.findChildren(QWidget):
            self._apply_widget_palette(child)
            try:
                child.setStyleSheet(shell_stylesheet)
            except Exception:
                pass

    def _ensure_shell(self) -> None:
        if not self.is_supported() or self._dialog is not None:
            return

        self._kernel_manager = QtInProcessKernelManager()
        self._kernel_manager.start_kernel(show_banner=False)
        self._kernel_manager.kernel.gui = "qt"
        self._kernel_client = self._kernel_manager.client()
        self._kernel_client.start_channels()
        self._register_command_magics()

        self._widget = RichJupyterWidget()
        self._widget.kernel_manager = self._kernel_manager
        self._widget.kernel_client = self._kernel_client
        self._widget.banner = self._banner
        self._apply_shell_theme()

        self._dialog = _ShellDialog(self.parent())
        self._dialog.setWindowTitle("Interactive Shell")
        self._dialog.resize(960, 640)
        self._dialog.visibility_changed.connect(self.visibility_changed.emit)
        self._apply_widget_palette(self._dialog)

        layout = QVBoxLayout(self._dialog)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._widget)

    def _push_context(
        self,
        bb: Any,
        sm: Any,
        current_state: Any = None,
        last_state: Any = None,
        commands: Optional[dict[str, Callable[[], Any]]] = None,
        active_path: Any = None,
        last_transition: Any = None,
    ) -> None:
        if self._kernel_manager is None:
            return

        proxy = SafeBlackboardProxy(bb)
        command_descriptions = {
            "next": "Execute Play Once and pause again.",
            "step": "Alias for next.",
            "cont": "Resume or start execution.",
            "play": "Alias for cont.",
            "pause": "Request a pause at the next transition.",
            "cancel_state": "Cancel the currently active state.",
            "cancel_sm": "Cancel the complete state machine.",
            "restart": "Restart the runtime after completion.",
            "where": "Print the current runtime path and last transition.",
        }
        self._commands = {
            name: _RuntimeShellCommand(
                name, callback, command_descriptions.get(name, name)
            )
            for name, callback in (commands or {}).items()
        }
        namespace = {
            "bb": proxy,
            "sm": sm,
            "current_state": current_state,
            "last_state": last_state,
            "active_path": tuple(active_path or tuple()),
            "last_transition": last_transition,
            "BlackboardAccessError": BlackboardAccessError,
            "SafeBlackboardProxy": SafeBlackboardProxy,
            "py_next": builtins.next,
            **self._commands,
        }
        self._kernel_manager.kernel.shell.push(namespace)
