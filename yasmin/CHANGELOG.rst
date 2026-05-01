^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^


5.1.0 (2026-05-01)
------------------
* fix: store Python CallbackSignal as native shared_ptr across blackboard (`#116 <https://github.com/uleroboticsgroup/yasmin/issues/116>`_)
  * fix: store Python CallbackSignal as native shared_ptr across blackboard boundary
  * formatter
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Blackboard python copy constructor (`#115 <https://github.com/uleroboticsgroup/yasmin/issues/115>`_)
  * add Blackboard copy bindings and remapping isolation tests
  * use blackboard copy for shell and stop clearing remappings on transitions
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* add callback signal (`#113 <https://github.com/uleroboticsgroup/yasmin/issues/113>`_)
  * add callback signal
  * docs(readme): add CallbackSignal chapter with Python and C++ examples
  * docs(web): add Callback Signal page and sidebar links across docs
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Maik

5.0.0 (2026-01-14)
------------------
* Fix state machine cancel propagation and add hard cancel API (`#105 <https://github.com/uleroboticsgroup/yasmin/issues/105>`_)
  * Fix state machine cancel propagation and add hard cancel API
  * Update editor to use direct state-machine cancel API
  * formatter
  * dont cancel state machine in cancel_state
  * use new cancel_state_machine API in sigint_handler
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Refactor blackboard storage to isolate remappings in concurrent execution (`#101 <https://github.com/uleroboticsgroup/yasmin/issues/101>`_)
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* refactor: reorganize include directives and refine code structure across packages
  Reorganize include statements by grouping system and third-party headers at the top with angle brackets, relocate project headers appropriately, and eliminate redundant or unused includes. Additionally, streamline source files by repositioning self-include statements, adjust comment formatting, correct minor typos, and remove superfluous whitespace or blank lines for improved readability and consistency without altering functionality.
* add remapping-aware blackboard iteration (`#100 <https://github.com/uleroboticsgroup/yasmin/issues/100>`_)
  * add remapping-aware blackboard iteration API with tests
  * return None for non-python blackboard values instead of unsafe casting
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Adding state metadata (`#95 <https://github.com/uleroboticsgroup/yasmin/issues/95>`_)
  * feat(state): add metadata support with default value injection for blackboard keys
  Add StateMetadata and BlackboardKeyInfo structures to enable states to declare
  input and output keys with optional default values. When a state is executed,
  default values are automatically injected into the blackboard for any missing
  input keys that have defaults defined. This allows states to declare their
  dependencies and provide sensible defaults without requiring the blackboard to
  be pre-populated. Includes Python bindings and comprehensive unit tests.
  * feat: add metadata and defaults support to editor and factory
  Add description and default value support to the state machine editor GUI and factory. This includes:
  - New DefaultsDialog for editing default blackboard values in the editor
  - Updated dialogs to support description and defaults for states and state machines
  - XML parsing and serialization for description attribute and Default elements
  - Plugin info now exposes description and input/output keys from states
  - Python bindings simplified to support arbitrary types for default values
  - External metadata storage preserves ABI compatibility with existing plugins
  * feat: add default value injection for Python blackboard keys
  Enables storing Python objects as default values for blackboard keys by using
  shared_ptr to maintain object lifetime, with a lambda that properly injects
  the default value using GIL acquisition when the key is accessed.
  * State metadata demos (`#77 <https://github.com/uleroboticsgroup/yasmin/issues/77>`_)
  * add state metadata across all python demo files
  * add description-aware constructors to BlackboardKeyInfo
  Throws an segfault currently.
  * add state metadata to all cpp demos
  * guard unsupported metadata default conversion in pybind
  * add cli tool for inspecting states
  Current counter value stored in the blackboard.
  Output keys:
  - counter
  * use keyword description in python demos to avoid pybind overload ambiguity
  * remove defaults for output_keys in python demos
  output_keys shouldn't have default values
  * only print keys if they exist
  * disambiguate string default metadata in state tests
  * make cpp_formatter happy
  * make cpp_formatter happy 2
  * make python_formatter happy
  * fix(pybind): make input key description keyword-only and remove default values for output keys
  This affects only python atm
  * remove add_output_key overload with default_value
  * remove py::kw_only
  * fix test
  * remove default value for output keys in tests
  * remove default values for output_keys in demos
  * fix demos
  * fix rosdep issue
  * fix metadata bindings and restore BlackboardKeyInfo description constructors
  * changing argument order of add_input_key to (key_name, description, default_value)
  making description mandatory if you want to supply a default_value
  * fix python formatting
  * remove copy paste error
  * typo
  * add description
  * update state.pyi
  * fix(pybind): fix string defaults
  * fix factory
  * refactor(metadata): unify BlackboardKeyInfo constructor API and fix default handling
  - enforce constructor order: (key_name, description, default_value)
  - remove ambiguous 2-argument default constructors
  - use std::decay_t for safe default value storage
  - fix const char* handling to store as std::string
  - update pybind layer to use explicit description argument
  - update factory to pass description before default value
  - simplify string handling by removing manual py::str special case
  * fix argument order
  * add default for cpp demo
  * add ros2 yasmin factory verb with recursive xml completion
  * filter factory xml completion by StateMachine root tag
  * add test verb to test a single state in an isolated state machine
  * move shared completers to package-level helper
  * fix formatter
  * fix formatter
  * add license
  * rename factory verb to run
  * split inspect into list and info
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * fix(cli): standardize CLI argument naming to kebab-case
  Standardize the `--disable_viewer_pub` argument to `--disable-viewer-pub` in run and test verbs to follow kebab-case convention for CLI arguments. Also add progress hiding support to plugin loading and use os._exit() for proper exit code propagation.
  * refactor(plugins): replace find_plugin with build_plugin_info for faster lookup
  Introduce build_plugin_info function to directly create PluginInfo objects
  from plugin names, avoiding XML file parsing overhead. Updated CLI components
  in completer.py and test.py to use the new function for improved performance.
  * Cli edit/viewer verb (`#78 <https://github.com/uleroboticsgroup/yasmin/issues/78>`_)
  * add startup CLI arg for opening XML file
  * add edit verb to open XML files in yasmin_editor
  * load startup xml only after canvas viewport is ready
  * make edit verb accept optional xml file and validate xml file
  * add viewer verb that starts yasmin_viewer_node
  ros2 yasmin viewer -h
  usage: ros2 yasmin viewer [-h] [--host HOST] [--port PORT]
  [--msg-per-second MSG_PER_SECOND]
  Start the YASMIN viewer web server
  options:
  -h, --help            show this help message and exit
  --host HOST           Viewer host address
  --port PORT           Viewer port
  --msg-per-second MSG_PER_SECOND
  Viewer update rate
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * store and restore node positions from xml (`#81 <https://github.com/uleroboticsgroup/yasmin/issues/81>`_)
  * store and restore node positions from xml
  * python formatting
  * python formatting
  * python formatting
  * fix nested outcome xml restore by loading container outcomes after child layout
  * add positions to demo state machines
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Faster plugin discovery (`#84 <https://github.com/uleroboticsgroup/yasmin/issues/84>`_)
  * add cached yasmin plugins manager package
  * formatter
  * use new plugin manager
  * add relative path for xml plugins
  befor we only stored package and file name but not the relative path
  inside the package
  * cleanup: remove leftover preload metadata parameter
  * improve doc strings
  * refactor: use ament plugin resources for yasmin cpp discovery
  * improve doc string
  * fix: load plugin metadata fields independently
  * add verbose plugin dump option to discovery node
  * formatter
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Cleaner default value dialog in editor (`#83 <https://github.com/uleroboticsgroup/yasmin/issues/83>`_)
  * move key descriptions into the description box to have a cleaner default dialog
  * python formatting
  * only show supported types in the defaults dialog
  * don't assume default type is string
  * merge outcomes and description box
  * formatter
  * add metadata support for outcome descriptions
  * switch -> to .
  * cpp formatter
  * add some desc to demos
  * print outcome description in cli info
  * add outcome_descriptions to state_properties_dialog
  * remove empty spaces
  * python formatter
  * add editable final outcome descriptions with xml persistence
  * formatter
  * load outcome description from xml
  * formatting
  * remove trailing white space
  * add outcome description to demo state machines
  * fix: load nested xml plugin states from StateMachine tags
  * formatter
  * feat: add blackboard key management ui and xml key support
  * copy paste issue
  * formatting
  * formatting
  * replace old syntax with new one in demo state machine
  * update to new key syntax
  * formatting
  * support explicit no-default type and empty string defaults for blackboard keys
  * formatter
  * apply nested container blackboard remappings to child states
  * compose parent and child blackboard remappings in nested state machines
  * formatting
  * show blackboard key default value and type in sidebar
  * Allow nested state machines to parse blackboard keys
  * remove old arg
  * remove old arg
  * remove old arg
  * auto derive the blackboard keys from all states
  * formatter
  * formatter
  * formatter
  * use correct keys
  * remove old arg
  * remove unused blackboard keys after graph sync
  * rename button to highlight
  * formatter
  * add keys to demos
  * restore blackboard remappings after state machine execution
  * formatter
  * show blackboard key default value
  * Add XML input overrides and completion for yasmin run
  * formatter
  * make types of key lower case
  * convert uppercase types to lowercase
  * convert type to lower
  * remove composing of remapping since its already handled by the factory
  * remove test
  * formatter
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * add validate CLI verb (`#88 <https://github.com/uleroboticsgroup/yasmin/issues/88>`_)
  * add validate CLI verb using factory create_sm_from_file and validate(strict_mode)
  * fix: isolate batch XML validation in subprocesses to avoid segfaults
  * dont print path twice
  * dont print file twice
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * fix(validate): destroy state machine explicitly before returning (`#89 <https://github.com/uleroboticsgroup/yasmin/issues/89>`_)
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Editor refactoring (`#86 <https://github.com/uleroboticsgroup/yasmin/issues/86>`_)
  * move key descriptions into the description box to have a cleaner default dialog
  * python formatting
  * only show supported types in the defaults dialog
  * don't assume default type is string
  * merge outcomes and description box
  * formatter
  * add metadata support for outcome descriptions
  * switch -> to .
  * cpp formatter
  * add some desc to demos
  * print outcome description in cli info
  * add outcome_descriptions to state_properties_dialog
  * remove empty spaces
  * python formatter
  * add editable final outcome descriptions with xml persistence
  * formatter
  * load outcome description from xml
  * formatting
  * remove trailing white space
  * add outcome description to demo state machines
  * fix: load nested xml plugin states from StateMachine tags
  * formatter
  * feat: add blackboard key management ui and xml key support
  * copy paste issue
  * formatting
  * formatting
  * replace old syntax with new one in demo state machine
  * update to new key syntax
  * formatting
  * support explicit no-default type and empty string defaults for blackboard keys
  * formatter
  * apply nested container blackboard remappings to child states
  * compose parent and child blackboard remappings in nested state machines
  * formatting
  * show blackboard key default value and type in sidebar
  * Allow nested state machines to parse blackboard keys
  * remove old arg
  * remove old arg
  * remove old arg
  * auto derive the blackboard keys from all states
  * formatter
  * formatter
  * formatter
  * use correct keys
  * remove old arg
  * remove unused blackboard keys after graph sync
  * rename button to highlight
  * formatter
  * add keys to demos
  * restore blackboard remappings after state machine execution
  * formatter
  * show blackboard key default value
  * Add XML input overrides and completion for yasmin run
  * formatter
  * make types of key lower case
  * convert uppercase types to lowercase
  * convert type to lower
  * remove composing of remapping since its already handled by the factory
  * remove test
  * formatter
  * add editor model layer, XML IO, and CLI print command
  * add recursive validation for state machines and concurrences
  * add license
  * require outcomes on state machines and concurrences
  * sync GUI backend through new model layer
  * remove xml_manager and clean duplicated model/io wiring
  * reduce gui/model duplication and move graph mutations into model
  * add container navigation view and render one level at a time
  * deduplicate nested transitions and scope blackboard view to current container
  * estore nested container transitions and persist nested keys
  * remove zip
  * restore loaded outcomes and improve transition editing/navigation
  * update container transition owners on rename and tighten fit margin
  * add readonly external xml viewer and render xml states as boxes
  * remove shadowed duplicate methods
  * formatting
  * Fix leaked nested remap keys in parent blackboard view
  * Fix stale blackboard keys after remap changes
  * add test sm for quicker debugging
  * added some transitions
  * fix self-loop anchors and hide fully-assigned ports
  * group duplicate transitions and make labels selectable
  * add label double-click transition rewiring
  * fix transition z-order so arrows stay behind boxes
  * remove transparency from container node brushes
  * Add color palette support and a dark mode
  * add first draft of a runtime mode
  * add improvements
  * improve runtime
  * remove blackboard remappings during pause
  * Improve runtime status/log styling and document runtime backend
  * Fix runtime log colors for light mode
  * add runtime test fsm
  * add description
  * fix runtime play-once pause handling
  * move test into test dir
  * refactor runtime logging bridge
  * remove some docs
  * fix and improve logger
  * remove print
  * add logs for start and end cb and rename transition color to system color
  * add log tree for nested state machines
  * fix transitions
  * copy paste issue
  * removed unused dialog
  * Use Add State dialog for sidebar state insertion with preselected plugin
  * Add cursor-based pending placement for new nodes
  * Refactor runtime logging into dedicated logging module
  * Refactor YasminEditor into focused mixin modules
  * collect mixin in another dir
  * group nodes in a dir
  * fix imports
  * remove stubs
  * split create_ui into focused helper methods
  * refactor: extract UI builder helpers into focused modules
  * refactor: split colors.py into theme modules and keep compatibility wrapper
  * refactor: extract shared container dialog base for state machine and concurrence dialogs
  * remove "optional" in dialogs because it took to much space
  * refactor: extract connection line geometry and label helpers into dedicated modules
  * add final outcome renaming with transition/reference updates
  * add runtime auto-follow for active nested states
  * highlight final root transition
  * add paused-transition interactive runtime shell
  * rename env var from YASMIN_EDITOR_COLOR to YASMIN_EDITOR_THEME
  * add concurrence to runtime test
  * expose current_state and last_state in interactive shell
  * factory: fix concurrence parsing in python
  * change runtime test order
  * stop expanding concurrence containers in runtime view
  * scope toolbar styling to editor toolbar to fix file dialog icons
  * refactor runtime controls and lock runtime mode during active execution
  * make interactive shell available after completion
  * fix: use editor XML state name in breadcrumb navigation
  * black formatter
  * Keep persistent runtime blackboard and enable shell only when not running
  * keep terminal active path instead of recomputing it on finish
  * remove dead functions
  * forbid state/final-outcome name collisions
  * add text blocks so devs can write documentation inside state machines
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Nested remappings (`#90 <https://github.com/uleroboticsgroup/yasmin/issues/90>`_)
  * compose nested blackboard remappings across state machines
  * improve test
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * add configure and local state parameters (`#91 <https://github.com/uleroboticsgroup/yasmin/issues/91>`_)
  * add state configure hook and local parameter blackboard
  * add parameter to plugin_info
  * add recursive configure and child parameter mappings
  * add parameters to xml plugin info
  * formatting
  * add editor support for parameter declarations and child parameter overwrites
  * factory: parse Param and ParamRemap from XML
  * formatter
  * fix: replace Python 3.9+ type annotation with typing.Dict for Foxy/Galactic
  * add state parameters and parameter remapping examples
  * fix: preserve Python state parameter metadata in C++ factory wrapper
  * add generic ROS serialization/deserialization states and fsm's
  * formatter
  * formatter
  * formatter
  * skip python states that require constructor arguments
  * test: add source dir to PYTHONPATH for yasmin_factory C++ gtests
  * remove ament_target_dependencies and keep plain target linking
  * ui: normalize displayed C++ metadata types for plugin info
  * cli: add parameter overrides to run/test and print params with param remaps
  * test: add ROS serialization state coverage and fix C++ interface type validation
  * preserve bytes and bytearray in Python blackboard wrapper
  * formatter
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Update docs (`#94 <https://github.com/uleroboticsgroup/yasmin/issues/94>`_)
  * docs: update main_concepts
  document blackboard key metadata and default input values
  add correct parameters section for state-local parameter storage
  * docs: add state metadata section to main concepts
  * docs: update xml state types chapter with list and dict blackboard support
  * docs: document configure lifecycle in state concept section
  * remove link
  * copy paste issue
  * docs: add YASMIN CLI documentation page
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  * Blackboard pywrapper support for homogeneous lists and dictionaries (`#93 <https://github.com/uleroboticsgroup/yasmin/issues/93>`_)
  * add state configure hook and local parameter blackboard
  * add parameter to plugin_info
  * add recursive configure and child parameter mappings
  * add parameters to xml plugin info
  * formatting
  * add editor support for parameter declarations and child parameter overwrites
  * factory: parse Param and ParamRemap from XML
  * formatter
  * fix: replace Python 3.9+ type annotation with typing.Dict for Foxy/Galactic
  * add state parameters and parameter remapping examples
  * fix: preserve Python state parameter metadata in C++ factory wrapper
  * add generic ROS serialization/deserialization states and fsm's
  * formatter
  * formatter
  * formatter
  * skip python states that require constructor arguments
  * test: add source dir to PYTHONPATH for yasmin_factory C++ gtests
  * remove ament_target_dependencies and keep plain target linking
  * ui: normalize displayed C++ metadata types for plugin info
  * cli: add parameter overrides to run/test and print params with param remaps
  * test: add ROS serialization state coverage and fix C++ interface type validation
  * preserve bytes and bytearray in Python blackboard wrapper
  * formatter
  * blackboard: add typed list/dict pywrapper conversions with documented dispatch
  * test: add blackboard coverage for typed lists, tuples and dicts
  * blackboard: add typed list/dict metadata conversion
  * factory: parse list/dict defaults from XML JSON values
  * cli: support list/dict input and parameter overrides
  * editor: add list/dict default type options and JSON hints
  * plugins: normalize vector/map metadata to list/dict types
  * formatteer
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
  ---------
  Co-authored-by: Maik <67489355+Maik13579@users.noreply.github.com>
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Add bytes support to Python blackboard wrapper for cross-language ROS interface exchange (`#76 <https://github.com/uleroboticsgroup/yasmin/issues/76>`_)
  * Add Python bytes support to blackboard wrapper
  This adds binary data exchange between Python and C++ via
  `bytes` and `std::vector<uint8_t>`.
  Plan (enable ROS interface exchange between Pyhton and C++):
  - serialize ROS interfaces
  - store them as byte arrays in the blackboard
  - deserialize them again in Python or C++
  * add demo for testing
  * add Copyright
  * add Copyright
  * move interface serialization to yasmin_ros (C++)
  * Document byte-array support for cross-language blackboard communication
  * Extend README with cross-language ROS interface serialization
  * add type
  * remove section and change format
  * update TOC
  * fix link
  * change format
  * add empty space to demo code
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Miguel Ángel González Santamarta

4.2.3 (2026-01-06)
------------------
* Setting C++ standard to 17
* Applying new make_share
* Expanding ptr macros of types
* Contributors: Miguel Ángel González Santamarta

4.2.2 (2025-12-28)
------------------
* minor fix in C++ header guards
* Contributors: Miguel Ángel González Santamarta

4.2.1 (2025-12-19)
------------------
* minor fix in C++ header guards
* Contributors: Miguel Ángel González Santamarta

4.2.0 (2025-12-17)
------------------
* Adding unit tests for state machine callbacks
* removing list of args form state machine callbacks
* new macro YASMIN_PTR_ALIASES for all pointer aliases
* Add new type aliases and macros to types.hpp and sync documentation examples with demo code
  - Add YASMIN_UNIQUE_PTR_ALIAS and YASMIN_WEAK_PTR_ALIAS macros
  - Add alias for Outcomes, Transitions, Remappings and more
  - Update README.md and tutorial docs to use Blackboard::SharedPtr instead of std::shared_ptr<Blackboard> to match actual demo implementations
* using unordered_map for values and registry of blackboard
* optimizing C++ code (adding const and noexcept)
* setting as private set_status and get_status in State
* removing repeated checks in blackboard pywrapper
* setting virtual destructor for state and removing __del__ from state machine pybind
* Contributors: Miguel Ángel González Santamarta

4.1.0 (2025-12-07)
------------------
* adding python-dev in deps and removing unused test deps for python
* setting signing handle as false by default
* adding SIGINT handler for YASMIN state machines in core package
* adding abort to fibonacci demo server
* Contributors: Miguel Ángel González Santamarta

4.0.2 (2025-12-01)
------------------
* fixing macros using pybind and setting EventsExecutor for Kilted and Rolling
* improving comments in C++
* removing directory in yasmin
* fixing comments in blackboard
* removing blackboard_value to simplify blackboard
* Contributors: Miguel Ángel González Santamarta

4.0.1 (2025-11-25)
------------------

4.0.0 (2025-11-24)
------------------
* adding destructor to state machine and cleanup to viewer pub
* adding attr test to python blackboard
* removing comments from python stub files of yasmin package
* Merge pull request `#70 <https://github.com/uleroboticsgroup/yasmin/issues/70>`_ from uleroboticsgroup/pybinding
  [New Great Version] Pybinding + Plugins + Editor
* adding set and get attr to blackboard pybinding
* setting input/output remappings
* fixing clang version
* stub files for yasmin pybind11
* new tests for state machine name and remappings in factory
* adding const&
* adding const&
* removing unused namespace yasmin
* adding name to SM class for root SM
* treating python list, dict, tuples and set as py::object in set function of Python blackboard
* fixing Python blackboard set to much data types
* remapping renamed to remappings
* adding type registry toblackboard
* replacing static constexpr bool with static bool in BlackboardPyWrapper
* fixing BlackboardPyWrapper to improve the type check in Python
* initial files fo yasmin_factory
* fixing license name in package.xml files
* fixing python logging
* adding python logging
* pybind11-dev for package.xml
* buildtool_depend for pybind11
* pybind checks for foxy
* python3-pybind11for package.xml
* adding pybind11-vendor to yasmin pacakge.xml
* fixing yasmin cmakelists
* yasmin pybindings created
* Contributors: Miguel Ángel González Santamarta

3.5.1 (2025-10-23)
------------------
* test exception messages in C++ tests
* creating C++ tests
* fixing python tests to run using colcon
* fixing exceptions in C++ version
* Contributors: Miguel Ángel González Santamarta

3.5.0 (2025-10-12)
------------------
* fixing Python comments
* adding support for concurrence states in the viewer
* Concurrence: replacing State set for a map of str to State
* fixing yasmin::State::cancel_state() order
* fixing cancel_state order
* Fix StateMachines cancels, waiting for states (`#65 <https://github.com/uleroboticsgroup/yasmin/issues/65>`_)
  Co-authored-by: Ferry Schoenmakers <ferry.schoenmakers@nobleo.nl>
* Fix re-entering concurrent states after a cancel by using the () operator instead of execute() function (`#64 <https://github.com/uleroboticsgroup/yasmin/issues/64>`_)
  * Fix re-entering concurrent states after a cancel by using the () operator instead of execute() function
  * Cleanup old, unused flags to avoid confusion
  ---------
  Co-authored-by: Ferry Schoenmakers <ferry.schoenmakers@nobleo.nl>
* Contributors: Ferry Schoenmakers, Miguel Ángel González Santamarta

3.4.0 (2025-08-25)
------------------
* fixing python remapping
* adding status to states
* wait for a state to cancel the state machine if running
* fixing flags names in python state
* set_current_state function for state machine
* adding publisher demo to README
* Fixing set log level in Cpp (`#61 <https://github.com/uleroboticsgroup/yasmin/issues/61>`_)
* Contributors: Miguel Ángel González Santamarta, Simone Morettini

3.3.0 (2025-06-28)
------------------
* ✨: Allowing that the cb_state receive arguments (`#59 <https://github.com/uleroboticsgroup/yasmin/issues/59>`_)
  * ⚡️: Allowing that the cb_state receive arguments
  * ✏️ Fixing typos
  * 🚨 fixing compilation error
  * ✏️ Fixing typos
* fixing yasmin logs and using them in ros versions
* Fix deprecation of ament_target_dependencies
* fixing start_state in fsm execute
* Pass current state instead of start state to transition cb (`#54 <https://github.com/uleroboticsgroup/yasmin/issues/54>`_)
* Contributors: Jfsslemos, Miguel Ángel González Santamarta, Paul Verhoeckx

3.2.0 (2025-04-11)
------------------
* Fix python YASMIN_LOG_ERROR method (`#51 <https://github.com/uleroboticsgroup/yasmin/issues/51>`_)
* fixing c++ version in CMakeLists
* Add Concurrence - Running Parallel/Concurrent States (`#50 <https://github.com/uleroboticsgroup/yasmin/issues/50>`_)
  * add concurrence state
  * add mutex for intermedaite state dict
  * prefix self vars with underscore
  * add concurrence str definition
  * add comment docs conform 120 character limit
  * fix init comment on concurrence
  * fix str gen for concurrence
  * replace state string representation as outcome map key with an integer alternative
  * add logic and tests to protect against running a state instance concurrently with itself
  * add newlines at end of files
  * verify intermedaite state outcome registration
  * add untested cpp implementation
  * fix compile warnings and add a demo
  * add licensing
  * run clang format on c files
  * run black format on py files
  * add python demo for concurrence
  * adjust to match python demo better
  ---------
  Co-authored-by: William Freidank <william.freidank@gtri.gatech.edu>
* Set remappings as empty dict instead of None (`#49 <https://github.com/uleroboticsgroup/yasmin/issues/49>`_)
* adding remapping to C++ version
* Feat/remmaping (`#47 <https://github.com/uleroboticsgroup/yasmin/issues/47>`_)
  * started remmaping
  * remmaping working with demo
  * reformated with black
  * formated again with --line-length 90
  * added remap documentation to the code
* Contributors: LuisMilczarek, Miguel Ángel González Santamarta, Noel Jiménez García, William Freidank

3.1.0 (2025-02-05)
------------------
* new yasmin logs
* improving c++ logs code
* setting default log level of python yasmin to info
* log levels added to yasmin logs
* fixing license comments
* Contributors: Miguel Ángel González Santamarta

3.0.3 (2024-12-17)
------------------
* hpp ifndef guards fixed
* fixing run_pytests install in CMakeLists.txt
* Contributors: Miguel Ángel González Santamarta

3.0.2 (2024-12-16)
------------------

3.0.1 (2024-12-01)
------------------
* minor fix to log from python state machine
* Contributors: Miguel Ángel González Santamarta

3.0.0 (2024-11-24)
------------------
* fixing CMakeLists format
* checking if state name is an outcome
* unittest2 removed from packages
* python-unittest2 and python3-pytest-cov added
* strict_mode added to validation
* setting validated to false when changing start state
* fixing C++ comments
* validated flag for state machine
* setting black formatter line length to 90
* comments for Python attributes moved to __init__
* python attribute comments inside __init__
* fixing documentation
* improving Python comments for Doxygen
* comments added to C++ files for Doxygen
* ifdef for cxxabi.h in state
* is_running added to state
* raise exception when state machine ends by canceling but with bad transition
* state fixes
* state machine str only state labels and type
* converting outcomes from list to set
* fixing state logs
* logs for cancel state
* fixing logs
* improving yasmin logs
* c++ to_string fixed
* more logs for the state machine execute
* short logs (by short file) and short UUID for yasmin_node
* state machine callbacks created
* replacing generic python exceptions
* exceptions for set_start_state
* C++ state machine validation
* raise exception when error appears in validate
* file, function and line added to yasmin logs
* new ros logs for YASMIN_LOG
* set_loggers to change yasmin logs
* debug logger added
* rclcpp and rclpy removed from yasmin pkg
* comments for terminal outcomes
* initial validation for sate machines in python
* black formatter applied to python files
* shared_pt creations fixed in demos
* fix(cancel): thread safety + allow overriding properly (`#27 <https://github.com/uleroboticsgroup/yasmin/issues/27>`_)
* stdexcept included for std errors
* Contributors: Miguel Ángel González Santamarta, Rein Appeldoorn

2.4.2 (2024-10-03)
------------------
* Made throw exceptions more understandable (`#24 <https://github.com/uleroboticsgroup/yasmin/issues/24>`_)
* Contributors: Cihat Kurtuluş Altıparmak

2.4.1 (2024-08-09 15:27)
------------------------

2.4.0 (2024-08-09 13:22)
------------------------

2.3.1 (2024-07-22)
------------------
* package version added
* Contributors: Miguel Ángel González Santamarta

2.3.0 (2024-07-13)
------------------
* Python blackboard fixed
  blackboard should be used os in cpp, that is as a dictionary
* Contributors: Miguel Ángel González Santamarta

2.2.0 (2024-06-30)
------------------
* lock/mutex added to blackboard
* translated_outcome removed
* yasmin_logs
* ros2 distros removed from yasmin package
* Contributors: Miguel Ángel González Santamarta

2.1.1 (2024-06-08)
------------------

2.1.0 (2024-06-05)
------------------
* jazzy distro fixed
* distros added to yasmin and yasmin_demo packages
* Contributors: Miguel Ángel González Santamarta

2.0.2 (2024-05-05)
------------------

2.0.1 (2024-04-16 13:57:56 +0200)
---------------------------------

2.0.0 (2024-04-16 13:57:56 +0200)
---------------------------------
* formatting fix
* upper removed
* Contributors: Miguel Ángel González Santamarta

1.0.0 (2023-12-06)
------------------
* possible outcomes added to exception
* python concat str fixed
* typing fixes
* blackboard included in yasmin __init__
* license added to files
* upper calls removed
* boost removed
* fixed c++ state machine execute
  Former-commit-id: 169f1af5561dfbc122f8b05350ea45eb3ad41264
* mutex/lock for current_state
  Former-commit-id: f606c66951f25aec11cac2088bb3eb5be6d4b5c1
* check if outcome belongs to current state
  Former-commit-id: 569e852eb8e5dfa96806b343642ea4dd591fa6af
* run_pytests fixed
  Former-commit-id: 1bc212e87b3993db78b84ab5872bb54c95462aab
* tests fixed
  Former-commit-id: a7360738b44ef1c29f9f1253dcc6badd14fe200b
* yasmin tests
  Former-commit-id: f069c2a613c0dfbc08f950cb9f095b5a6473cc50
* ament_export_dependencies(${DEPENDENCIES}) added
  Former-commit-id: e003ff4860318beb62066e98e48e339c3995f6af
* C++ version created
  Former-commit-id: 9d02daf711aaaf25d36b0b58284c2e9dc5f053e0
* ' replaced by "
  Former-commit-id: 63c642bbfc6975a63c4e742c3573d8db901e40ac
* 1 version of yasmin
  Former-commit-id: a17c6a35baaa77099292a87ed2dd65587fe4e01b
* Contributors: Miguel Ángel González Santamarta
