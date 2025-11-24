^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_factory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
  [New Greate Version] Pybinding + Plugins + Editor
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
