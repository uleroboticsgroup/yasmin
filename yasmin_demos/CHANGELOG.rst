^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
