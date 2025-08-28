^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
* comments for Python attributes moved to __init\_\_
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
* minor formating fixes
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
* blackboard included in yasmin __init\_\_
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
