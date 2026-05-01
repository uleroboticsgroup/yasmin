^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


5.1.0 (2026-05-01)
------------------

5.0.0 (2026-01-14)
------------------
* refactor: update package share path retrieval for compatibility with rclcpp version checks
* refactor: replace get_package_share_path with get_package_share_directory for compatibility
* refactor: update includes to support conditional package share path retrieval
* refactor: reorganize include directives and refine code structure across packages
  Reorganize include statements by grouping system and third-party headers at the top with angle brackets, relocate project headers appropriately, and eliminate redundant or unused includes. Additionally, streamline source files by repositioning self-include statements, adjust comment formatting, correct minor typos, and remove superfluous whitespace or blank lines for improved readability and consistency without altering functionality.
* Add ament_index_cpp dependency and update state creation in yasmin_factory
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
* Fix ActionState demo and documentation (`#74 <https://github.com/uleroboticsgroup/yasmin/issues/74>`_)
* Contributors: Andreas Kohler, Miguel Ángel González Santamarta

4.2.3 (2026-01-06)
------------------
* Setting C++ standard to 17
* Applying new make_share
* Removing times argument from monitor demo
* Contributors: Miguel Ángel González Santamarta

4.2.2 (2025-12-28)
------------------

4.2.1 (2025-12-19)
------------------

4.2.0 (2025-12-17)
------------------
* creating alias for Parameters and fixing style of concurrence demo
* Add new type aliases and macros to types.hpp and sync documentation examples with demo code
  - Add YASMIN_UNIQUE_PTR_ALIAS and YASMIN_WEAK_PTR_ALIAS macros
  - Add alias for Outcomes, Transitions, Remappings and more
  - Update README.md and tutorial docs to use Blackboard::SharedPtr instead of std::shared_ptr<Blackboard> to match actual demo implementations
* fixing comments and log order in demos
* using createUnmanagedInstance in pybind_bridge to allow Python mange the lifetime of objects
* improving Python + cleaning Python viewer pub on shutdown
* setting as private set_status and get_status in State
* improving comments on foo and bar states from demos
* removing cleanup from Python viewer publisher
* adding class_libraries to plugins
* setting default value for PrintOdometryState times in demos
* Contributors: Miguel Ángel González Santamarta

4.1.0 (2025-12-07)
------------------
* adding python-dev in deps and removing unused test deps for python
* setting signing handle as false by default
* updating nav demo in docs
* adding SIGINT handler for YASMIN state machines in core package
* adding Reentrant as default callback group in ActionState
* adding abort to fibonacci demo server
* Contributors: Miguel Ángel González Santamarta

4.0.2 (2025-12-01)
------------------
* fixing python comments
* removing directory in yasmin
* Contributors: Miguel Ángel González Santamarta

4.0.1 (2025-11-25)
------------------
* fixing dependencies in package.xml of demos and factory
* Contributors: Miguel Ángel González Santamarta

4.0.0 (2025-11-24)
------------------
* adding cleanup to python demos
* removing resource directory from demos
* moving #include <pluginlib/class_list_macros.hpp> to the bottom of files
* removing nav_demo due to rolling buildfarm
* fixing C++ format
* removing comments from main functions
* creating C++ nav_demo
* Merge pull request `#70 <https://github.com/uleroboticsgroup/yasmin/issues/70>`_ from uleroboticsgroup/pybinding
  [New Great Version] Pybinding + Plugins + Editor
* fixing Python imports
* fixing clang version
* cancling SM created by factoy in C++
* removing unused namespace yasmin
* adding name to SM class for root SM
* fixing Python blackboard set to much data types
* creating tests for yasmin_factory
* C++ factory
* fixing yasmin factory functions names
* initial files fo yasmin_factory
* fixing license name in package.xml files
* Contributors: Miguel Ángel González Santamarta

3.5.1 (2025-10-23)
------------------

3.5.0 (2025-10-12)
------------------
* fixing C++ demos
* fixing Python comments
* Concurrence: replacing State set for a map of str to State
* Contributors: Miguel Ángel González Santamarta

3.4.0 (2025-08-25)
------------------
* fixing python remapping
* fixing logs and viewer names in demos and README
* parameters state added
* adding publisher demo to README
* Contributors: Miguel Ángel González Santamarta

3.3.0 (2025-06-28)
------------------
* python version of publisher state
* improving publisher demo
* initial version of a publisher state
* removing FOXY env and fixing service qos for kilted and greater
* Kilted support (`#56 <https://github.com/uleroboticsgroup/yasmin/issues/56>`_)
  * Kilted support
  * Keep only the single Foxy exception
  * fixup! Keep only the single Foxy exception
* minor format fixes to multiple_states_demo
* adding kilted flag to cmakelists
* Fix deprecation of ament_target_dependencies
* Declare states outside the state_machine (`#53 <https://github.com/uleroboticsgroup/yasmin/issues/53>`_)
  * Create .h for foo and bar states
  * Create foo and bar state and the states handler
  * feat/multiple_states_demo example
  * Fix the C++ format
  * Create python version for multiple states demo
  * Fix C++ format pt.2
  * Add license to new files
* Contributors: Miguel Ángel González Santamarta, Pedro Edom, Tim Clephas

3.2.0 (2025-04-11)
------------------
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
* adding remapping to C++ version
* Preempt monitor state on cancel request (`#46 <https://github.com/uleroboticsgroup/yasmin/issues/46>`_)
  * Preempt monitor state on cancel request
  * Add canceled outcome to monitor_state and monitor_demo
  * Implement monitor_state cancel check in Python
* Feat/remmaping (`#47 <https://github.com/uleroboticsgroup/yasmin/issues/47>`_)
  * started remmaping
  * remmaping working with demo
  * reformated with black
  * formated again with --line-length 90
  * added remap documentation to the code
* Contributors: LuisMilczarek, Miguel Ángel González Santamarta, Paul Verhoeckx, William Freidank

3.1.0 (2025-02-05)
------------------
* set_ros_loggers allows setting node to log
* fixing license comments
* updating changelog files
* Contributors: Miguel Ángel González Santamarta

3.0.3 (2024-12-17)
------------------
* fixing license in c++ service client demo
* removing unused params in demo servers
* Contributors: Miguel Ángel González Santamarta

3.0.2 (2024-12-16)
------------------
* adding servers for demos
* build_depend for ros_environment
* adding ros-environment dependency
* fixing dependencies in package.xml files
* action-tutorials-interfaces replaced with example-interfaces
* Contributors: Miguel Ángel González Santamarta

3.0.1 (2024-12-01)
------------------

3.0.0 (2024-11-24)
------------------
* fixing CMakeLists format
* ROS 2 fixed in CMakeLists
* fixing C++ comments
* setting black formatter line length to 90
* improving Python comments for Doxygen
* comments added to C++ files for Doxygen
* galactic support added to cmakelists
* is_running added to state
* raise exception when state machine ends by canceling but with bad transition
* catching KeyboardInterrupt in python demos
* cancel C++ state machine on ROS 2 shutdown
* validating state machine before publishing
* improving yasmin logs
* c++ to_string fixed
* new ros logs for YASMIN_LOG
* fixing demos names in cpp demo and monitor
* black formatter applied to python files
* shared_pt creations fixed in demos
* Contributors: Miguel Ángel González Santamarta

2.4.2 (2024-10-03)
------------------

2.4.1 (2024-08-09 15:27)
------------------------

2.4.0 (2024-08-09 13:22)
------------------------

2.3.1 (2024-07-22)
------------------
* demos fixes
* yasmin_demo renamed to yasmin_demos
* Contributors: Miguel Ángel González Santamarta

2.3.0 (2024-07-13)
------------------

2.2.0 (2024-06-30)
------------------

2.1.1 (2024-06-08)
------------------

2.1.0 (2024-06-05)
------------------

2.0.2 (2024-05-05)
------------------

2.0.1 (2024-04-16 13:57:56 +0200)
---------------------------------

2.0.0 (2024-04-16 13:57:56 +0200)
---------------------------------

1.0.0 (2023-12-06)
------------------
