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

HELP_HTML = """
<h2>YASMIN Editor Help</h2>

<p>
The YASMIN Editor is used to build, inspect, and run hierarchical state machines.
It supports regular states, nested state machines, concurrences, final outcomes,
text notes, blackboard inspection, a temporary shelf, and runtime execution.
</p>

<h3>1. Basic workflow</h3>
<ul>
  <li><b>New / Open / Save / Save As</b>: create or work with XML state machine files.</li>
  <li><b>Add</b> menu or empty-canvas <b>right-click</b>: add states, nested state machines, concurrences, final outcomes, and text blocks.</li>
  <li>New items are created in a pending placement mode. <b>Left-click</b> places them. <b>Esc</b> or <b>right-click</b> cancels placement.</li>
  <li>The breadcrumb bar shows the current container path. Click a breadcrumb to jump there. Use <b>Fit</b> to reframe the current view.</li>
</ul>

<h3>2. Canvas controls</h3>
<ul>
  <li><b>Mouse wheel</b>: zoom.</li>
  <li><b>Middle mouse button</b>: pan.</li>
  <li><b>Back mouse button</b>: go up one container level.</li>
  <li><b>Rubber-band drag</b>: select multiple items.</li>
  <li><b>Drag selected items</b>: move them inside the current container.</li>
  <li><b>Delete / Backspace</b>: delete the current selection.</li>
</ul>

<h3>3. Adding and editing states</h3>
<ul>
  <li>The left sidebar provides filterable lists for <b>Python States</b>, <b>C++ States</b>, and <b>XML State Machines</b>.</li>
  <li><b>Double-click</b> a plugin entry to add it to the current container.</li>
  <li><b>Double-click</b> a state on the canvas to edit its properties.</li>
  <li><b>Right-click</b> a state for context actions such as edit, delete, or enter.</li>
  <li>State properties include name, plugin, description, remappings, and parameter overwrites.</li>
</ul>

<h3>4. Containers</h3>
<ul>
  <li><b>State Machine</b>: sequential container with child states, transitions, outcomes, and a start state.</li>
  <li><b>Concurrence</b>: parallel container that collects child-state outcomes into concurrence outcomes.</li>
  <li><b>Double-click</b> a container to edit it.</li>
  <li><b>Ctrl + double-click</b> a container to enter it.</li>
  <li>When editing a state machine, its <b>Start State</b> must reference one of its child states.</li>
  <li>When editing a concurrence, its <b>Default Outcome</b> must reference one of its final outcomes.</li>
</ul>

<h3>5. External XML state machines</h3>
<ul>
  <li>XML states can reference external state machine files.</li>
  <li>Entering such a state loads the referenced XML and opens it as a nested view.</li>
  <li>External XML views are <b>read-only</b>.</li>
  <li>If the XML file cannot be resolved or loaded, the editor shows an error instead of entering it.</li>
</ul>

<h3>6. Final outcomes and transitions</h3>
<ul>
  <li><b>Final Outcome</b>: exit point of the current container.</li>
  <li>Create transitions by dragging from a node's <b>blue connection port</b> onto another node or final outcome.</li>
  <li>If a state has multiple available outcomes, the editor asks which one should trigger the transition.</li>
  <li><b>Double-click</b> a transition label to start rewiring its target.</li>
  <li>In a normal <b>State Machine</b>, final outcomes of the current container cannot start transitions.</li>
  <li>In a <b>Concurrence</b>, child states may only connect to final outcomes of that same concurrence.</li>
</ul>

<h3>7. Shelf and selection workflows</h3>
<ul>
  <li><b>Ctrl+C</b>: copy the current selection and place the copy manually on the main canvas.</li>
  <li><b>Ctrl+E</b>: extract the selected states into a new nested state machine.</li>
  <li><b>Toggle Shelf</b> (<b>Ctrl+Shift+B</b>): show or hide the right-side shelf dock.</li>
  <li><b>Drag selected items to the shelf</b>: temporarily store them there.</li>
  <li><b>Drag items from the shelf back to the main canvas</b>: restore them into the current container.</li>
  <li>The shelf has its own <b>Fit</b> and <b>Clear</b> actions.</li>
</ul>

<h3>8. Text blocks</h3>
<ul>
  <li><b>Add Text</b>: create a free-form note inside the current container.</li>
  <li><b>Double-click</b> a text block to edit it inline.</li>
  <li><b>Ctrl+B</b> and <b>Ctrl+I</b> insert Markdown-style bold and italic markers.</li>
  <li><b>Ctrl+Enter</b> finishes editing. <b>Esc</b> cancels the current text edit.</li>
  <li>Outside edit mode, text blocks are shown as a rendered preview.</li>
</ul>

<h3>9. Blackboard panel</h3>
<ul>
  <li>The blackboard panel lists known keys for the current model tree.</li>
  <li>Use the filter box to quickly find a key.</li>
  <li>Selecting a key highlights the states that use it.</li>
  <li><b>Double-click</b> a blackboard key to view or edit its metadata.</li>
  <li>Keys starting with <code>.</code> are hidden by default. Use <b>Hidden: On</b> to show them.</li>
</ul>

<h3>10. Runtime mode</h3>
<ul>
  <li><b>Runtime Mode</b> runs the current state machine from a temporary XML snapshot.</li>
  <li>While runtime mode is active, the editor canvas is <b>read-only</b>.</li>
  <li>The runtime control bar provides:
    <ul>
      <li><b>Play</b>: start or resume execution.</li>
      <li><b>Request Pause</b>: pause at the next transition.</li>
      <li><b>Play Once</b>: execute exactly one state and pause again.</li>
      <li><b>Cancel State</b>: request cancellation of the active state.</li>
      <li><b>Cancel State Machine</b>: request cancellation of the full runtime.</li>
      <li><b>Restart</b>: rebuild the runtime from a fresh XML snapshot after execution has finished.</li>
      <li><b>Auto Follow</b>: automatically enter nested containers so the active state stays visible.</li>
      <li><b>Interactive Shell</b>: open a shell with runtime context when <code>qtconsole</code> is available.</li>
    </ul>
  </li>
  <li>During runtime, <b>right-click</b> a state to add or remove a breakpoint before that state.</li>
  <li>The left sidebar switches from plugin lists to a runtime log view with selectable log level.</li>
</ul>

<h3>11. Read-only situations</h3>
<ul>
  <li>The editor becomes read-only in <b>Runtime Mode</b>.</li>
  <li>The editor also becomes read-only when browsing an <b>external XML</b> subtree.</li>
  <li>In read-only mode, editing actions are disabled, but properties can still be viewed.</li>
</ul>

<h3>12. Validation</h3>
<p>
The model validator checks structure and references across the full state machine tree.
Typical checks include:
</p>
<ul>
  <li>state names must not be empty,</li>
  <li>outcome names and blackboard keys must be unique per state,</li>
  <li>child-state names must not collide with final-outcome names in the same container,</li>
  <li>state machines must define at least one outcome,</li>
  <li>state machines with multiple child states need a valid start state,</li>
  <li>transition targets must exist,</li>
  <li>Python states require <code>module</code> and <code>class_name</code>,</li>
  <li>C++ states require <code>class_name</code>,</li>
  <li>XML states require <code>file_name</code>,</li>
  <li>concurrences validate default outcome and outcome-map references.</li>
</ul>

<h3>13. Practical tips</h3>
<ul>
  <li>Use the plugin filters and blackboard filter when working with larger models.</li>
  <li>Use <b>Fit</b> often after entering containers or moving large selections.</li>
  <li>Use the shelf to temporarily move subgraphs out of the way without deleting them.</li>
  <li>Use text blocks for local design notes and documentation directly on the canvas.</li>
  <li>If a container or state only opens in view mode, you are likely in runtime mode or inside an external XML view.</li>
</ul>
""".strip()
