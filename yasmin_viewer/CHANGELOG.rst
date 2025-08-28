^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


3.4.0 (2025-08-25)
------------------

3.3.0 (2025-06-28)
------------------
* fixing yasmin logs and using them in ros versions
* Fix deprecation of ament_target_dependencies
* Contributors: Miguel Ángel González Santamarta

3.2.0 (2025-04-11)
------------------
* fixing c++ version in CMakeLists
* Contributors: Miguel Ángel González Santamarta

3.1.0 (2025-02-05)
------------------
* updating yasmin viewer dependencies
* new viewer build
* adding package-lock.json to gitignore
* package-lock.json removed
* adding layouts to toolbar
* fixing license comments
* Contributors: Miguel Ángel González Santamarta

3.0.3 (2024-12-17)
------------------
* hpp ifndef guards fixed
* Contributors: Miguel Ángel González Santamarta

3.0.2 (2024-12-16)
------------------
* fixing dependencies in package.xml files
* Contributors: Miguel Ángel González Santamarta

3.0.1 (2024-12-01)
------------------

3.0.0 (2024-11-24)
------------------
* fixing C++ comments
* started removed from viewer node
* setting black formatter line length to 90
* comments for Python attributes moved to __init\_\_
* fixing documentation
* improving Python comments for Doxygen
* comments added to C++ files for Doxygen
* check nodes and edges before building graph in viewer
* converting outcomes from list to set
* validating state machine before publishing
* set_loggers to change yasmin logs
* zoom feature added to yasmin_viewer
* black formatter applied to python files
* fix(viewer): graceful shutdown (`#37 <https://github.com/uleroboticsgroup/yasmin/issues/37>`_)
* feat(viewer): print http:// link (`#35 <https://github.com/uleroboticsgroup/yasmin/issues/35>`_)
  So that it is clickable from console
* fix(viewer): added missing rosdeps (`#36 <https://github.com/uleroboticsgroup/yasmin/issues/36>`_)
* Contributors: Miguel Ángel González Santamarta, Rein Appeldoorn

2.4.2 (2024-10-03)
------------------
* minor style fixes
* Merge pull request `#23 <https://github.com/uleroboticsgroup/yasmin/issues/23>`_ from CihatAltiparmak/fix/convert_to_ros_node_shared_ptr
  Converted rclcpp::Node& to rclcpp::Node::SharedPtr
* Converted some node parameters to const ref
* Converted rlccpp::Node& to rclcpp::Node::SharedPtr
* Contributors: CihatAltiparmak, Miguel Ángel González Santamarta

2.4.1 (2024-08-09 15:27)
------------------------
* current_hidden_fsm fixed in viewer
* Contributors: Miguel Ángel González Santamarta

2.4.0 (2024-08-09 13:22)
------------------------
* new viewer build
* hidden fsm represented as octagon in viewer
* show only active FSMs added to viewer
* Contributors: Miguel Ángel González Santamarta

2.3.1 (2024-07-22)
------------------
* package version added
* Contributors: Miguel Ángel González Santamarta

2.3.0 (2024-07-13)
------------------

2.2.0 (2024-06-30)
------------------

2.1.1 (2024-06-08)
------------------

2.1.0 (2024-06-05)
------------------
* viewer app bar + hide nested fsm
* Contributors: Miguel Ángel González Santamarta

2.0.2 (2024-05-05)
------------------

2.0.1 (2024-04-16 13:57:56 +0200)
---------------------------------

2.0.0 (2024-04-16 13:57:56 +0200)
---------------------------------
* Merge pull request `#15 <https://github.com/uleroboticsgroup/yasmin/issues/15>`_ from uleroboticsgroup/no_simple_node
  No simple node
* no simple_node C++ version
* python free of simple_node
* Contributors: Miguel Ángel González Santamarta

1.0.0 (2023-12-06)
------------------
* Merge pull request `#11 <https://github.com/uleroboticsgroup/yasmin/issues/11>`_ from aminballoon/add-ros-args-yasmin-viewer-node
  Add ros args yasmin viewer node for custom host and port
* change type param port str->int
* add ros args
* self loop edges fixed
* typing fixes
* license added to files
* interfaces to msgs
* upper calls removed
* new yasmin viewer information
  more fsm nested can be view
  now state machines are arrays of states
  the first element is the fsm
* nodes size adjusted in viewer
  Former-commit-id: 1f1bccef16d40c329637b4e9c72626ff44d48c58
* SM final outcomes shape fixed in viewer
  Former-commit-id: f5c4d849e9e8f39f79ea5a31d6f12cdf4b53ea39
* yasmin_viewer updated
  Former-commit-id: 9a88fb0d1a0ca9b78fc6b8a74c3af7a81582331f
* yasmin tests
  Former-commit-id: f069c2a613c0dfbc08f950cb9f095b5a6473cc50
* yasmin iterfaces fixed
  Former-commit-id: 95dd4cfd21b430afdc8497e8d51a69ee2bbf4573
* viewer fixed
  Former-commit-id: ba248d977e1e5c6da11d882e07ed053f2f20cd39
* C++ version created
  Former-commit-id: 9d02daf711aaaf25d36b0b58284c2e9dc5f053e0
* ' replaced by "
  Former-commit-id: 63c642bbfc6975a63c4e742c3573d8db901e40ac
* viewer named changed + rebuild
  Former-commit-id: caa80be23dab6b29a986e62f7544a07991e978f3
* 1 version of yasmin
  Former-commit-id: a17c6a35baaa77099292a87ed2dd65587fe4e01b
* Contributors: Miguel Ángel González Santamarta, PannapatC
