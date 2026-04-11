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
<h2>YASMIN Editor - Quick Guide</h2>
<h3>File Operations</h3>
<b>New/Open/Save:</b> Create, load, or save state machines from XML files.
<h3>Building State Machines</h3>
<b>State Machine Name:</b> Set root name.<br>
<b>Start State:</b> Select initial state.<br>
<b>Add State:</b> Add regular state (Python/C++/XML).<br>
<b>Add State Machine:</b> Add nested container.<br>
<b>Add Concurrence:</b> Add parallel container.<br>
<b>Add Final Outcome:</b> Add an exit point. Reusing the same final-outcome name creates another visual alias for the same logical outcome.<br>
<b>Add Text:</b> Add free-form documentation blocks with inline editing.
<h3>Working with States</h3>
<b>Double-click:</b> Plugin to add state.<br>
<b>Right-click:</b> State options.<br>
<b>Drag:</b> Reposition states.<br>
<b>Rubber-band drag:</b> Multi-select states, outcomes, transitions, and text blocks.<br>
<b>Delete Selected:</b> Remove all selected items.<br>
<b>Text Blocks:</b> Double-click to edit inline, use Ctrl+B / Ctrl+I for Markdown markers.
<h3>Shelf Workflow</h3>
<b>Ctrl+C:</b> Copy the selected subgraph and place the copy manually on the canvas.<br>
<b>Drag to Shelf:</b> Drag the current selection onto the right-side shelf to store it temporarily.<br>
<b>Drag from Shelf:</b> Drag stored items back onto the main canvas to restore them.<br>
<b>Extract:</b> Ctrl+E extracts the selected states into a nested state machine.<br>
<b>Shelf Panel:</b> Toggle the right-side shelf split view, use Fit to recenter and zoom its view without changing the shelf width, and use it as a temporary storage space between edits.
<h3>Creating Transitions</h3>
<b>Drag from blue port:</b> Create transitions.<br>
<b>Select outcome:</b> Choose trigger.
<h3>Containers</h3>
<b>Nested States:</b> Ctrl + double-click to enter, double-click to edit.<br>
<b>Final Outcomes:</b> Exit points.<br>
<b>State Machine:</b> Sequential.<br>
<b>Concurrence:</b> Parallel.
<h3>Canvas Navigation</h3>
<b>Scroll:</b> Zoom.<br>
<b>Middle mouse:</b> Pan.
<h3>Validation</h3>
• Name set<br>
• Start state selected<br>
• Final outcome exists<br>
• Unique child-state names
<h3>Tips</h3>
• Use filters to find states.<br>
• Containers auto-resize.<br>
• Concurrence states transition to internal final outcomes.<br>
• XML SMs are regular states.
""".strip()
