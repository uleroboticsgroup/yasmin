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

from typing import Callable

from PyQt5.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QPushButton,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)


def build_left_panel(editor) -> QWidget:
    """Create the complete left panel."""
    left_panel = QWidget()
    left_panel.setObjectName("leftPanel")
    left_panel.setMinimumWidth(0)
    left_layout = QVBoxLayout(left_panel)

    editor.blackboard_widget = build_blackboard_widget(editor)
    left_layout.addWidget(editor.blackboard_widget)

    editor.editor_sidebar_widget = build_editor_sidebar_widget(editor)
    left_layout.addWidget(editor.editor_sidebar_widget)

    editor.runtime_sidebar_widget = build_runtime_sidebar_widget(editor)
    left_layout.addWidget(editor.runtime_sidebar_widget)

    return left_panel


def build_blackboard_widget(editor) -> QWidget:
    """Create the blackboard panel."""
    widget = QWidget()
    widget.setObjectName("blackboardPanel")
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)

    layout.addWidget(QLabel("<b>Blackboard Keys:</b>"))

    editor.blackboard_filter = QLineEdit()
    editor.blackboard_filter.setObjectName("blackboardFilterEdit")
    editor.blackboard_filter.setPlaceholderText("Filter blackboard keys...")
    editor.blackboard_filter.textChanged.connect(editor.filter_blackboard_keys)
    layout.addWidget(editor.blackboard_filter)

    editor.blackboard_list = QListWidget()
    editor.blackboard_list.setObjectName("blackboardList")
    editor.blackboard_list.setSelectionMode(QAbstractItemView.SingleSelection)
    editor.blackboard_list.itemSelectionChanged.connect(
        editor.on_blackboard_selection_changed
    )
    editor.blackboard_list.itemDoubleClicked.connect(editor.edit_selected_blackboard_key)
    layout.addWidget(editor.blackboard_list)

    button_row = QHBoxLayout()
    editor.highlight_blackboard_btn = QPushButton("Highlight: On")
    editor.highlight_blackboard_btn.setObjectName("highlightBlackboardButton")
    editor.highlight_blackboard_btn.setCheckable(True)
    editor.highlight_blackboard_btn.setChecked(True)
    editor.highlight_blackboard_btn.toggled.connect(editor.toggle_blackboard_highlighting)
    button_row.addWidget(editor.highlight_blackboard_btn)

    editor.show_hidden_blackboard_btn = QPushButton("Hidden: Off")
    editor.show_hidden_blackboard_btn.setObjectName("showHiddenBlackboardButton")
    editor.show_hidden_blackboard_btn.setCheckable(True)
    editor.show_hidden_blackboard_btn.setChecked(False)
    editor.show_hidden_blackboard_btn.toggled.connect(
        editor.toggle_hidden_blackboard_keys
    )
    editor.show_hidden_blackboard_btn.setToolTip(
        'Blackboard keys starting with "." are hidden by default.'
    )
    button_row.addWidget(editor.show_hidden_blackboard_btn)

    layout.addLayout(button_row)

    return widget


def build_editor_sidebar_widget(editor) -> QWidget:
    """Create the editor sidebar with available plugins."""
    widget = QWidget()
    widget.setObjectName("editorSidebarPanel")
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)

    editor.python_filter, editor.python_list = build_filterable_list_section(
        layout,
        "<b>Python States:</b>",
        "Filter Python states...",
        lambda value: editor.filter_list(editor.python_list, value),
        editor.on_plugin_double_clicked,
        filter_object_name="pythonPluginFilterEdit",
        list_object_name="pythonPluginList",
    )

    editor.cpp_filter, editor.cpp_list = build_filterable_list_section(
        layout,
        "<b>C++ States:</b>",
        "Filter C++ states...",
        lambda value: editor.filter_list(editor.cpp_list, value),
        editor.on_plugin_double_clicked,
        filter_object_name="cppPluginFilterEdit",
        list_object_name="cppPluginList",
    )

    editor.xml_filter, editor.xml_list = build_filterable_list_section(
        layout,
        "<b>XML State Machines:</b>",
        "Filter XML state machines...",
        lambda value: editor.filter_list(editor.xml_list, value),
        editor.on_xml_double_clicked,
        filter_object_name="xmlPluginFilterEdit",
        list_object_name="xmlPluginList",
    )

    return widget


def build_filterable_list_section(
    layout: QVBoxLayout,
    title: str,
    placeholder: str,
    filter_handler: Callable[[str], None],
    double_click_handler,
    *,
    filter_object_name: str,
    list_object_name: str,
) -> tuple[QLineEdit, QListWidget]:
    """Create a titled filter + list section."""
    layout.addWidget(QLabel(title))

    filter_edit = QLineEdit()
    filter_edit.setObjectName(filter_object_name)
    filter_edit.setPlaceholderText(placeholder)
    filter_edit.textChanged.connect(filter_handler)
    layout.addWidget(filter_edit)

    list_widget = QListWidget()
    list_widget.setObjectName(list_object_name)
    list_widget.itemDoubleClicked.connect(double_click_handler)
    layout.addWidget(list_widget)

    return filter_edit, list_widget


def build_runtime_sidebar_widget(editor) -> QWidget:
    """Create the runtime log sidebar."""
    widget = QWidget()
    widget.setObjectName("runtimeSidebarPanel")
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)

    header_layout = QHBoxLayout()
    header_layout.addWidget(QLabel("<b>Logs:</b>"))
    header_layout.addStretch()

    editor.runtime_log_level_combo = QComboBox()
    editor.runtime_log_level_combo.setObjectName("runtimeLogLevelCombo")
    editor.runtime_log_level_combo.setProperty("flatInput", True)
    editor.runtime_log_level_combo.addItems(["ERROR", "WARN", "INFO", "DEBUG"])
    editor.runtime_log_level_combo.setCurrentText("INFO")
    editor.runtime_log_level_combo.currentTextChanged.connect(
        editor.on_runtime_log_level_changed
    )
    header_layout.addWidget(editor.runtime_log_level_combo)
    layout.addLayout(header_layout)

    editor.runtime_log_view = QTextBrowser()
    editor.runtime_log_view.setObjectName("runtimeLogView")
    editor.runtime_log_view.setReadOnly(True)
    editor.runtime_log_view.setOpenExternalLinks(False)
    editor.runtime_log_view.setOpenLinks(False)
    editor.runtime_log_view.setLineWrapMode(QTextBrowser.NoWrap)
    editor.runtime_log_view.setProperty("viewerText", True)
    editor.runtime_log_view.document().setDocumentMargin(8)
    layout.addWidget(editor.runtime_log_view)

    widget.setVisible(False)
    return widget
