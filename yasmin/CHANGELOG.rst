^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^


4.0.0 (2025-11-24)
------------------
* adding destructor to state machine and cleanup to viewer pub
* adding attr test to python blackboard
* removing comments from python stub files of yasmin package
* Merge pull request `#70 <https://github.com/uleroboticsgroup/yasmin/issues/70>`_ from uleroboticsgroup/pybinding
  [New Greate Version] Pybinding + Plugins + Editor
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
* reverting only-one-workflow
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
* comments for Python attributes moved to __init\_\_
* python attribute comments inside __init\_\_
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
* blackboard included in yasmin __init\_\_
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
