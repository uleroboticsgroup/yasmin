# Copyright (C) 2026 Maik Knof
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

from typing import Callable, Tuple
from yasmin_editor.qt_compat import QtWidgets


def build_left_panel(editor) -> QtWidgets.QWidget:
    """Create the complete left panel."""
    left_panel = QtWidgets.QWidget()
    left_panel.setObjectName("leftPanel")
    left_panel.setMinimumWidth(0)
    left_layout = QtWidgets.QVBoxLayout(left_panel)

    editor.blackboard_widget = build_blackboard_widget(editor)
    left_layout.addWidget(editor.blackboard_widget)

    editor.editor_sidebar_widget = build_editor_sidebar_widget(editor)
    left_layout.addWidget(editor.editor_sidebar_widget)

    editor.runtime_sidebar_widget = build_runtime_sidebar_widget(editor)
    left_layout.addWidget(editor.runtime_sidebar_widget)

    return left_panel


def build_blackboard_widget(editor) -> QtWidgets.QWidget:
    """Create the blackboard panel."""
    widget = QtWidgets.QWidget()
    widget.setObjectName("blackboardPanel")
    layout = QtWidgets.QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)

    layout.addWidget(QtWidgets.QLabel("<b>Blackboard Keys:</b>"))

    editor.blackboard_filter = QtWidgets.QLineEdit()
    editor.blackboard_filter.setObjectName("blackboardFilterEdit")
    editor.blackboard_filter.setPlaceholderText("Filter blackboard keys...")
    editor.blackboard_filter.textChanged.connect(editor.filter_blackboard_keys)
    layout.addWidget(editor.blackboard_filter)

    editor.blackboard_list = QtWidgets.QListWidget()
    editor.blackboard_list.setObjectName("blackboardList")
    editor.blackboard_list.setSelectionMode(
        QtWidgets.QAbstractItemView.SelectionMode.SingleSelection
    )
    editor.blackboard_list.itemSelectionChanged.connect(
        editor.on_blackboard_selection_changed
    )
    editor.blackboard_list.itemDoubleClicked.connect(editor.edit_selected_blackboard_key)
    layout.addWidget(editor.blackboard_list)

    button_row = QtWidgets.QHBoxLayout()
    editor.highlight_blackboard_btn = QtWidgets.QPushButton("Highlight: On")
    editor.highlight_blackboard_btn.setObjectName("highlightBlackboardButton")
    editor.highlight_blackboard_btn.setCheckable(True)
    editor.highlight_blackboard_btn.setChecked(True)
    editor.highlight_blackboard_btn.toggled.connect(editor.toggle_blackboard_highlighting)
    button_row.addWidget(editor.highlight_blackboard_btn)

    editor.show_hidden_blackboard_btn = QtWidgets.QPushButton("Hidden: Off")
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


def build_editor_sidebar_widget(editor) -> QtWidgets.QWidget:
    """Create the editor sidebar with available plugins."""
    widget = QtWidgets.QWidget()
    widget.setObjectName("editorSidebarPanel")
    layout = QtWidgets.QVBoxLayout(widget)
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
    layout: QtWidgets.QVBoxLayout,
    title: str,
    placeholder: str,
    filter_handler: Callable[[str], None],
    double_click_handler,
    *,
    filter_object_name: str,
    list_object_name: str,
) -> Tuple[QtWidgets.QLineEdit, QtWidgets.QListWidget]:
    """Create a titled filter + list section."""
    layout.addWidget(QtWidgets.QLabel(title))

    filter_edit = QtWidgets.QLineEdit()
    filter_edit.setObjectName(filter_object_name)
    filter_edit.setPlaceholderText(placeholder)
    filter_edit.textChanged.connect(filter_handler)
    layout.addWidget(filter_edit)

    list_widget = QtWidgets.QListWidget()
    list_widget.setObjectName(list_object_name)
    list_widget.itemDoubleClicked.connect(double_click_handler)
    layout.addWidget(list_widget)

    return filter_edit, list_widget


def build_runtime_sidebar_widget(editor) -> QtWidgets.QWidget:
    """Create the runtime log sidebar."""
    widget = QtWidgets.QWidget()
    widget.setObjectName("runtimeSidebarPanel")
    layout = QtWidgets.QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)

    header_layout = QtWidgets.QHBoxLayout()
    header_layout.addWidget(QtWidgets.QLabel("<b>Logs:</b>"))
    header_layout.addStretch()

    editor.runtime_log_level_combo = QtWidgets.QComboBox()
    editor.runtime_log_level_combo.setObjectName("runtimeLogLevelCombo")
    editor.runtime_log_level_combo.setProperty("flatInput", True)
    editor.runtime_log_level_combo.addItems(["ERROR", "WARN", "INFO", "DEBUG"])
    editor.runtime_log_level_combo.setCurrentText("INFO")
    editor.runtime_log_level_combo.currentTextChanged.connect(
        editor.on_runtime_log_level_changed
    )
    header_layout.addWidget(editor.runtime_log_level_combo)
    layout.addLayout(header_layout)

    editor.runtime_log_view = QtWidgets.QTextBrowser()
    editor.runtime_log_view.setObjectName("runtimeLogView")
    editor.runtime_log_view.setReadOnly(True)
    editor.runtime_log_view.setOpenExternalLinks(False)
    editor.runtime_log_view.setOpenLinks(False)
    editor.runtime_log_view.setLineWrapMode(QtWidgets.QTextBrowser.LineWrapMode.NoWrap)
    editor.runtime_log_view.setProperty("viewerText", True)
    editor.runtime_log_view.document().setDocumentMargin(8)
    layout.addWidget(editor.runtime_log_view)

    widget.setVisible(False)
    return widget
