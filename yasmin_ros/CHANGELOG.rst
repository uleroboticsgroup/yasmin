^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


5.1.0 (2026-05-01)
------------------
* refactor(build): extract tf_buffer_state into separate plugin library
  Move tf_buffer_state.cpp from main library sources to a dedicated shared library target to avoid namespace collisions, update plugin configuration accordingly, and adjust test linkages.
* fix: create TfBufferState node during configure instead of constructor (`#112 <https://github.com/uleroboticsgroup/yasmin/issues/112>`_)
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Maik, Miguel Ángel González Santamarta

5.0.0 (2026-01-14)
------------------
* Fix YasminNode shutdown lifecycle and stabilize yasmin_ros teardown (`#107 <https://github.com/uleroboticsgroup/yasmin/issues/107>`_)
  * Fix YasminNode lifecycle and ROS client cache teardown in yasmin_ros
  * formatter
  * add missing includes
  * switch back to shared_ptr
  * Fix incomplete shared_ptr revert in ROS client cache
  * formatter
  * revert chagnes in cache
  * trigger tests for flaky check
  * revert last commit for third test run
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* fix: correct tf2_ros-py and tf2-py dependencies to tf2_ros_py and tf2_py in package.xml
* fix: add missing tf2_ros-py and tf2-py dependencies in package.xml
* add tf_buffer_state (`#104 <https://github.com/uleroboticsgroup/yasmin/issues/104>`_)
  * add tf_buffer_state
  * Use Foxy-compatible tf2_ros::Buffer constructor
  * wrong variable
  * revert to original yasmin node
  * remove buffer and listener from blackboard
  * Revert "remove buffer and listener from blackboard"
  This reverts commit 4c3b0138e3d09bb8a1a189733f7c859979653220.
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* refactor(yasmin_ros): extract serialization states into separate plugin libraries
  Move RosSerializeCppState and RosDeserializeCppState into individual shared libraries (ros_serialize_cpp_state and ros_deserialize_cpp_state) to avoid namespace collisions, updating CMakeLists.txt and plugins.xml accordingly. Adjust test linking to include the new libraries.
* feat(yasmin_ros): add outcome descriptions and output keys to GetParametersState
  - Add descriptions for SUCCEED and ABORT outcomes
  - Add output keys for each parameter to facilitate state transitions
* feat(yasmin_ros): add outcome descriptions and documentation to state classes
  Introduce descriptive text for state outcomes in action, monitor, publisher, and service states to clarify their meanings. Enhance Doxygen documentation for serialization state classes and interface handlers to improve code readability and API usability.
  These additions provide better context for developers using the states, aiding in debugging and state machine design.
* refactor: reorder include directives in headers and remove trailing blank lines in sources
  Affected files in yasmin_pcl and yasmin_ros packages to improve code organization and consistency.
* refactor: reorganize include directives and refine code structure across packages
  Reorganize include statements by grouping system and third-party headers at the top with angle brackets, relocate project headers appropriately, and eliminate redundant or unused includes. Additionally, streamline source files by repositioning self-include statements, adjust comment formatting, correct minor typos, and remove superfluous whitespace or blank lines for improved readability and consistency without altering functionality.
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
  - counter [default=0, type=int]
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
* Adapting tests for Foxy and Galactic
* Contributors: Miguel Ángel González Santamarta

4.2.1 (2025-12-19)
------------------
* minor fix in C++ header guards
* Adapting tests for Foxy and Galactic
* Contributors: Miguel Ángel González Santamarta

4.2.0 (2025-12-17)
------------------
* creating alias for Parameters and fixing style of concurrence demo
* Add new type aliases and macros to types.hpp and sync documentation examples with demo code
  - Add YASMIN_UNIQUE_PTR_ALIAS and YASMIN_WEAK_PTR_ALIAS macros
  - Add alias for Outcomes, Transitions, Remappings and more
  - Update README.md and tutorial docs to use Blackboard::SharedPtr instead of std::shared_ptr<Blackboard> to match actual demo implementations
* setting EventsExecutor for Jazzy
* setting the YasminNode constructor as protected for Singleton
* improving Python + cleaning Python viewer pub on shutdown
* optimizing C++ code (adding const and noexcept)
* minor fixes in Python comments and imports
* Contributors: Miguel Ángel González Santamarta

4.1.0 (2025-12-07)
------------------
* removing duplicated comments and setting generateUUID as inline
* adding python-dev in deps and removing unused test deps for python
* adding SIGINT handler for YASMIN state machines in core package
* creating lock and cs_status before while in monitor state
* adding abort to fibonacci demo server
* adding cancel checks in action state
* Contributors: Miguel Ángel González Santamarta

4.0.2 (2025-12-01)
------------------
* fixing macros using pybind and setting EventsExecutor for Kilted and Rolling
* fixing code format
* using macros for EventsExecutors
* fixing python comments
* improving comments in C++
* removing directory in yasmin
* Contributors: Miguel Ángel González Santamarta

4.0.1 (2025-11-25)
------------------
* setting timeout for yasmin_ros tests
* Contributors: Miguel Ángel González Santamarta

4.0.0 (2025-11-24)
------------------
* removing ament_target_dependencies form yasmin_ros
* Merge pull request `#70 <https://github.com/uleroboticsgroup/yasmin/issues/70>`_ from uleroboticsgroup/pybinding
  [New Great Version] Pybinding + Plugins + Editor
* adding rcl init to yasmin node
* fixing Python imports
* adding const&
* adding const&
* fixing const std::string & in yasmin_ros
* removing clear client cache from yasmin_ros C++ tests
* adding type registry toblackboard
* fixing c++ monitor state
* new yasmin_ros tests for each state
* fixing license name in package.xml files
* adding python logging
* Contributors: Miguel Ángel González Santamarta

3.5.1 (2025-10-23)
------------------
* renaming ROSCommunicationsCache to ROSClientsCache
* creating C++ tests
* fixing python tests to run using colcon
* Contributors: Miguel Ángel González Santamarta

3.5.0 (2025-10-12)
------------------
* ros communications cache created
* fixing timeout in C++ MonitorState
* fixing C++ demos
* fixing c++ timeout waits
* fixing timeout in ros2 states
* fixing Python comments
* fixing timeout params names
* fixing format
* Add More basic outcome: retry, add publisher into states, and set waiting-response timeout in service state. (`#67 <https://github.com/uleroboticsgroup/yasmin/issues/67>`_)
  * Add SkippableState for python-lib
  * Add basic outcome for Skippable State
  * Init Skippable State
  * State service server
  * add publisher to service state node
  * add publisher to service state node
  * add publisher to monitoring state
  * edit bug
  * fix upper case
  * add _srv_callback template
  * fix skipable state multi srv type
  * fix bug
  * fix bug wrong line
  * edit service and skippable state
  * add cancel features in yasmin-ros
  * add publisher to skip state
  * add publisher to all states
  * edit monitoring node
  * merege upstream/main
  * remove test installation + yasmin dep in yasmin-ros.xml
  * edit bug in skippable state
  * add abort in monitor_state
  * debug timeout in service server
  * add abort handler
  * edit service server
  * edit server state
  * comment cpp builder
  * update yasmin ros cpp lib
  * solve bug
  * (feat): add retries and action timeout in yasmin ros
  * update c++ code style, remove publisher in monitor state (python), and apply retry mechanism to python version
  * fix bug in ServiceState timeout, and add retry mechanism in pytest
  ---------
  Co-authored-by: PannapatC <PannapatC@arv.co.th>
  Co-authored-by: Pakapak <PakapakS@arv.co.th>
  Co-authored-by: Aminballoon <44831071+aminballoon@users.noreply.github.com>
  Co-authored-by: Miguel Ángel González Santamarta <mgons@unileon.es>
* fixing yasmin::State::cancel_state() order
* fixing cancel_state order
* Contributors: Miguel Ángel González Santamarta, Pakapak Silpapinun

3.4.0 (2025-08-25)
------------------
* fixing int64_t in get paramters state for foxy/galactic
* fixing get values of C++get parameters state
* parameters state added
* adding publisher demo to README
* adding missing cond.clear to monitor state
* improving monitor state by replacing sleeps with events and conds
* Contributors: Miguel Ángel González Santamarta

3.3.0 (2025-06-28)
------------------
* destroying subscriber in monitor state
* python version of publisher state
* initial version of a publisher state
* fixing yasmin logs and using them in ros versions
* checking rclcpp/version.h for older versions
* removing FOXY env and fixing service qos for kilted and greater
* adding callbackgroup to ros2 states
* fixing msg_queue in monitor state
* Kilted support (`#56 <https://github.com/uleroboticsgroup/yasmin/issues/56>`_)
  * Kilted support
  * Keep only the single Foxy exception
  * fixup! Keep only the single Foxy exception
* Add FAIL to basic outcomes (`#57 <https://github.com/uleroboticsgroup/yasmin/issues/57>`_)
  * Add FAIL to basic outcomes
  * add fail outcome to C
  * fixed for formatter checks
* adding kilted flag to cmakelists
* Fix deprecation of ament_target_dependencies
* Contributors: Gabriel Dorneles, Miguel Ángel González Santamarta, Tim Clephas

3.2.0 (2025-04-11)
------------------
* fixing c++ version in CMakeLists
* Preempt monitor state on cancel request (`#46 <https://github.com/uleroboticsgroup/yasmin/issues/46>`_)
  * Preempt monitor state on cancel request
  * Add canceled outcome to monitor_state and monitor_demo
  * Implement monitor_state cancel check in Python
* Contributors: Miguel Ángel González Santamarta, Paul Verhoeckx

3.1.0 (2025-02-05)
------------------
* new yasmin logs
* improving ros_logs
* set_ros_loggers allows setting node to log
* fixing format
* fixing license comments
* Contributors: Miguel Ángel González Santamarta

3.0.3 (2024-12-17)
------------------
* hpp ifndef guards fixed
* fixing run_pytests install in CMakeLists.txt
* Contributors: Miguel Ángel González Santamarta

3.0.2 (2024-12-16)
------------------
* build_depend for ros_environment
* adding ros-environment dependency
* fixing dependencies in package.xml files
* Contributors: Miguel Ángel González Santamarta

3.0.1 (2024-12-01)
------------------

3.0.0 (2024-11-24)
------------------
* fixing CMakeLists format
* ROS 2 fixed in CMakeLists
* unittest2 removed from packages
* python-unittest2 and python3-pytest-cov added
* strict_mode added to validation
* fixing C++ comments
* fixing C++ comments
* setting black formatter line length to 90
* comments for Python attributes moved to __init__
* fixing documentation
* improving Python comments for Doxygen
* comments added to C++ files for Doxygen
* galactic support added to cmakelists
* state machine str only state labels and type
* converting outcomes from list to set
* short logs (by short file) and short UUID for yasmin_node
* replacing generic python exceptions
* file, function and line added to yasmin logs
* new ros logs for YASMIN_LOG
* set_loggers to change yasmin logs
* black formatter applied to python files
* action state cancel fixed
* Contributors: Miguel Ángel González Santamarta

2.4.2 (2024-10-03)
------------------
* minor style fixes
* Merge pull request `#23 <https://github.com/uleroboticsgroup/yasmin/issues/23>`_ from CihatAltiparmak/fix/convert_to_ros_node_shared_ptr
  Converted rclcpp::Node& to rclcpp::Node::SharedPtr
* Converted some node parameters to const ref
* Converted rlccpp::Node& to rclcpp::Node::SharedPtr
* Contributors: CihatAltiparmak

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
* new feedback_handler for python ActionState
* ActionState log fixed
* Contributors: Miguel Ángel González Santamarta

2.2.0 (2024-06-30)
------------------
* feedback handler added to action state
* lock/mutex added to blackboard
* logs for monitor state
* action and service logs
* c++ service logs fixed
* ros2 logs for services
* Contributors: Miguel Ángel González Santamarta

2.1.1 (2024-06-08)
------------------
* wait_for_service timeout added to C++
* Merge pull request `#21 <https://github.com/uleroboticsgroup/yasmin/issues/21>`_ from santiregui/add_timeout_srv
  Added TIMEOUT in service_state
* Added TIMEOUT outcome and use it in the service_state for timeout interruption
* Contributors: Miguel Ángel González Santamarta, Santiago Iregui

2.1.0 (2024-06-05)
------------------
* jazzy distro fixed
* foxy fixed
* multi ros2 distro added to yasmin_ros
* Contributors: Miguel Ángel González Santamarta

2.0.2 (2024-05-05)
------------------
* Merge pull request `#17 <https://github.com/uleroboticsgroup/yasmin/issues/17>`_ from mattwilliamson/template-fix
  Fixing build error for Mac M1 / Apple clang version 14.0.3
* Fixing build error
* Contributors: Matt Williamson, Miguel Ángel González Santamarta

2.0.1 (2024-04-16 13:57:56 +0200)
---------------------------------

2.0.0 (2024-04-16 13:57:56 +0200)
---------------------------------
* Merge pull request `#15 <https://github.com/uleroboticsgroup/yasmin/issues/15>`_ from uleroboticsgroup/no_simple_node
  No simple node
* yasmin node singleton fixed
* yasmin_node dependencies fixed
* no simple_node C++ version
* python free of simple_node
* minor formatting fixes
* Contributors: Miguel Ángel González Santamarta

1.0.0 (2023-12-06)
------------------
* ActionState typo fixed
* new monitor state
* typing fixes
* Merge pull request `#7 <https://github.com/uleroboticsgroup/yasmin/issues/7>`_ from jkaniuka/fix/monitor_state_transitions
  Handling of undeclared transitions
* Handling of undeclared transitions
* action state fixed
* service_state fixed
* blackboard included in yasmin __init__
* license added to files
* simple_node updated
* boost removed
* fixed monitor msg variable name in blackboard
* monitor state created
* run_pytests fixed
  Former-commit-id: 1bc212e87b3993db78b84ab5872bb54c95462aab
* result callback fixed
  Former-commit-id: ef6daf09abd60e2b2331d6b2293fab97880dcc9a
* tests fixed
  Former-commit-id: a7360738b44ef1c29f9f1253dcc6badd14fe200b
* yasmin_ros tests created
  Former-commit-id: 2103ec64ea1dd1d2cdcb3a3158bb5ad0220f267c
* yasmin tests
  Former-commit-id: f069c2a613c0dfbc08f950cb9f095b5a6473cc50
* ament_export_dependencies(${DEPENDENCIES}) added
  Former-commit-id: e003ff4860318beb62066e98e48e339c3995f6af
* C++ version created
  Former-commit-id: 9d02daf711aaaf25d36b0b58284c2e9dc5f053e0
* typing fix
  Former-commit-id: 220570ad37526ae3e5c560f78ce4a2780309dd84
* ' replaced by "
  Former-commit-id: 63c642bbfc6975a63c4e742c3573d8db901e40ac
* custom_ros2 replaced by simple_node
  Former-commit-id: a33b916642d822f9990e3115f404e12f8033382e
* 1 version of yasmin
  Former-commit-id: a17c6a35baaa77099292a87ed2dd65587fe4e01b
* Contributors: Jan Kaniuka, Miguel Ángel González Santamarta
