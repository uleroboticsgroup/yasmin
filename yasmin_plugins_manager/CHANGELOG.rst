^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_plugins_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.1.0 (2026-05-01)
------------------
* fix plugin cache serialization for non-primitive defaults (`#114 <https://github.com/uleroboticsgroup/yasmin/issues/114>`_)
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Maik

5.0.0 (2026-01-14)
------------------
* normalize cpp metadata types in editor and plugin cache path (`#108 <https://github.com/uleroboticsgroup/yasmin/pull/108>`)
  * normalize cpp metadata types in editor and plugin cache path
  * fix metadata type normalization for python-loaded plugins and nested cpp templates
  * always normalize metadata
* build: update setup.py to exclude test packages and add pytest, remove linter tests
  Remove placeholder linter test files for copyright, flake8, and pep257
  in yasmin_cli and yasmin_plugins_manager. Update package configurations
  to use pytest as a test extra and exclude test directories from packages.
* test: replace placeholder tests with linter tests for copyright, flake8, and pep257 in yasmin_cli and yasmin_plugins_manager
  Remove placeholder test files and __init_\_.py from yasmin_cli and yasmin_plugins_manager test directories.
  Add actual linter test files for copyright, flake8, and pep257 compliance checks.
* test: add placeholder tests to yasmin_cli and yasmin_plugins_manager
  Remove tests_require from yasmin_cli setup.py to adjust testing dependencies.
* feat(discovery): add env, package.xml and XML-file ignore rules with cache invalidation (`#97 <https://github.com/uleroboticsgroup/yasmin/issues/97>`_)
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Update build configurations and README to include yasmin_pcl package
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
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Miguel Ángel González Santamarta
