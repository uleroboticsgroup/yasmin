^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_factory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.2.2 (2025-12-28)
------------------
* Adding enable_viewer_pub to factory nodes
* minor fix in C++ header guards
* Contributors: Miguel Ángel González Santamarta

4.2.1 (2025-12-19)
------------------
* minor fix in C++ header guards
* Contributors: Miguel Ángel González Santamarta

4.2.0 (2025-12-17)
------------------
* Add new type aliases and macros to types.hpp and sync documentation examples with demo code
  - Add YASMIN_UNIQUE_PTR_ALIAS and YASMIN_WEAK_PTR_ALIAS macros
  - Add alias for Outcomes, Transitions, Remappings and more
  - Update README.md and tutorial docs to use Blackboard::SharedPtr instead of std::shared_ptr<Blackboard> to match actual demo implementations
* using unordered_map for values and registry of blackboard
* fixing comments and log order in demos
* removing no-op deleted form pybind_bridge in factory
* using createUnmanagedInstance in pybind_bridge to allow Python mange the lifetime of objects
* improving Python + cleaning Python viewer pub on shutdown
* optimizing C++ code (adding const and noexcept)
* removing cleanup from Python viewer publisher
* removing repeated checks in blackboard pywrapper
* Contributors: Miguel Ángel González Santamarta

4.1.0 (2025-12-07)
------------------
* adding python-dev in deps and removing unused test deps for python
* setting signing handle as false by default
* adding SIGINT handler for YASMIN state machines in core package
* Contributors: Miguel Ángel González Santamarta

4.0.2 (2025-12-01)
------------------
* fixing macros using pybind and setting EventsExecutor for Kilted and Rolling
* removing directory in yasmin
* Contributors: Miguel Ángel González Santamarta

4.0.1 (2025-11-25)
------------------
* adding ament_cmake_python to factory package.xml
* fixing dependencies in package.xml of demos and factory
* Contributors: Miguel Ángel González Santamarta

4.0.0 (2025-11-24)
------------------
* adding boos dependency to factory for Foxy
* adding macros in factory for foxy and galactir
* removing cleanup from factory
* adding cleanup to python demos
* removing unused functions and attributes from pybind11_bridge
* moving #include <pluginlib/class_list_macros.hpp> to the bottom of files
* removing "state_machines" directory from factory
* replacing rcpputils with filesystem
* removing ament_python from dependencies
* removing comments from main functions
* Merge pull request `#70 <https://github.com/uleroboticsgroup/yasmin/issues/70>`_ from uleroboticsgroup/pybinding
  [New Great Version] Pybinding + Plugins + Editor
* adding file_name + package to include XML state machines
* fixing order to load included XML state machine in factory
* replacing xml with lxml in Python
* initial_state -> start_state
* adding initial state to factories
* removing parameters from factory
* fixing Python imports
* including other XML state machines in factory
* setting input/output remappings
* cancling SM created by factoy in C++
* new tests for state machine name and remappings in factory
* replacing from/to with old/new in factory
* fixing wrong remappings in python factory
* adding name to SM class for root SM
* adding remappings to factory
* fixing const std::string & in yasmin_ros
* adding yasmin_viewer to factory pacakge
* treating python list, dict, tuples and set as py::object in set function of Python blackboard
* C++ yasmin_factory_node
* fixing BlackboardPyWrapper to improve the type check in Python
* creating tests for yasmin_factory
* PythonStateHolder for C++ factory
* refactor python yasmin_factory
* C++ factory
* fixing yasmin factory functions names
* initial files fo yasmin_factory
* Contributors: Miguel Ángel González Santamarta
