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
from typing import Any, Optional

_MISSING = object()

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QDialog, QVBoxLayout

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
            }
        )


class _ShellDialog(QDialog):
    visibility_changed = pyqtSignal(bool)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self.visibility_changed.emit(True)

    def hideEvent(self, event) -> None:
        super().hideEvent(event)
        self.visibility_changed.emit(False)

    def closeEvent(self, event) -> None:
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
        self._banner = (
            "YASMIN interactive shell\n"
            "\n"
            "bb -> safe blackboard proxy\n"
            "sm -> root state machine\n"
            "\n"
            "Use bb['key'] or bb.get('key').\n"
            "Attribute-style access is disabled to keep IPython introspection stable.\n"
            "Values that cannot be converted from C++ are returned as\n"
            "BlackboardAccessError instead of raising immediately.\n"
            "\n"
            "Close this shell before resuming execution.\n"
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

    def open_shell(self, bb: Any, sm: Any) -> None:
        self._ensure_shell()
        self._push_context(bb=bb, sm=sm)

        if self._dialog is None:
            return

        self._dialog.show()
        self._dialog.raise_()
        self._dialog.activateWindow()

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

    def _ensure_shell(self) -> None:
        if not self.is_supported() or self._dialog is not None:
            return

        self._kernel_manager = QtInProcessKernelManager()
        self._kernel_manager.start_kernel(show_banner=False)
        self._kernel_manager.kernel.gui = "qt"
        self._kernel_client = self._kernel_manager.client()
        self._kernel_client.start_channels()

        self._widget = RichJupyterWidget()
        self._widget.kernel_manager = self._kernel_manager
        self._widget.kernel_client = self._kernel_client
        self._widget.banner = self._banner

        self._dialog = _ShellDialog(self.parent())
        self._dialog.setWindowTitle("Interactive Shell")
        self._dialog.resize(960, 640)
        self._dialog.visibility_changed.connect(self.visibility_changed.emit)

        layout = QVBoxLayout(self._dialog)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._widget)

    def _push_context(self, bb: Any, sm: Any) -> None:
        if self._kernel_manager is None:
            return

        proxy = SafeBlackboardProxy(bb)
        namespace = {
            "bb": proxy,
            "sm": sm,
            "BlackboardAccessError": BlackboardAccessError,
            "SafeBlackboardProxy": SafeBlackboardProxy,
        }
        self._kernel_manager.kernel.shell.push(namespace)
