^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_pcl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.0 (2026-07-07)
------------------
* relicense to Apache 2.0 (`#122 <https://github.com/uleroboticsgroup/yasmin/issues/122>`_)
* refactor: update minimum CMake version to 3.10 across multiple CMakeLists.txt files
* Enhance documentation and improve code clarity across multiple modules
  - Added detailed comments and descriptions for constructors, methods, and parameters in various classes, including `Concurrence`, `StateMachine`, and filter states in `yasmin_pcl`.
  - Improved logging initialization comments in `logs.cpp` for better understanding of log level settings.
  - Updated `blackboard_pybind11.cpp` to clarify the registration of `BlackboardPyWrapper`.
  - Enhanced the `yasmin_demos` states with additional private member documentation for clarity on their purpose.
  - Improved the `yasmin_factory` class documentation, particularly around blackboard key and parameter mappings.
  - Added utility functions in `test_utils.hpp` for creating point clouds and temporary file paths with clear descriptions.
  - Enhanced the `ros_clients_cache` and `supported_interface_serialization` classes with detailed comments on cache management and serialization callbacks.
  - Improved the `YasminNode` and `YasminViewerNode` classes with comments on copy prevention and connection handling.
  - General code cleanup and formatting improvements for better readability and maintainability.
* refactor: use 'this' pointer for member variable access in multiple state classes
* Refactor and enhance yasmin packages
  - Removed unnecessary destructors from PclToRosPointCloud2State and RosToPclPointCloud2State classes.
  - Added cancellation checks in SavePcdState and SavePlyState execute methods.
  - Improved ActionState to notify when action is done and handle cancellation more effectively.
  - Updated basic_outcomes to use inline constexpr for string constants.
  - Enhanced MonitorState and ServiceState to include cancellation checks and improved locking mechanisms.
  - Refactored ROSClientsCache to use per-map locks for thread safety.
  - Updated get_parameters_state to handle int64_t instead of int for parameter declaration.
  - Improved YasminViewerNode to manage active HTTP connections and handle server shutdown more gracefully.
  - Refactored yasmin_viewer_pub to use unordered_map for better performance and added checks for rclpy state before publishing.
* style: format code for improved readability in multiple files
* refactor: refactor and optimize yasmin code
  - Removed unnecessary code and improved the formatting of plugin headers in discovery_node.py.
  - Simplified metadata loading in plugin_info.py by introducing a safe_get method to handle exceptions.
  - Enhanced plugin manager to streamline package retrieval and signature validation.
  - Cleaned up ROS interface serialization by removing unused includes.
  - Updated action client state to utilize a retry mechanism for server connections.
  - Improved service state handling with a retry mechanism for service calls.
  - Consolidated node resolution logic into a utility function for better code reuse.
  - Refactored viewer node to enhance readability and maintainability.
  - Updated tests to reflect changes in expected warning log counts.
  - Added new utility functions for retry logic and node resolution in ros_state_utils.py.
* fix: update input_indices for VoxelGridState test to include only specific points
* Contributors: Miguel Ángel González Santamarta

5.1.0 (2026-05-01)
------------------

5.0.0 (2026-01-14)
------------------
* refactor(yasmin_pcl): downgrade log levels from error to warning in point cloud states
  Change YASMIN_LOG_ERROR calls to YASMIN_LOG_WARN in execute methods across filters and io modules to reduce severity of null pointer and missing key reports, allowing processes to continue with warnings instead of aborting on recoverable issues.
  This adjustment improves robustness in development environments by treating input validation failures as warnings rather than fatal errors.
* refactor: reorder include directives in headers and remove trailing blank lines in sources
  Affected files in yasmin_pcl and yasmin_ros packages to improve code organization and consistency.
* refactor: reorganize include directives and refine code structure across packages
  Reorganize include statements by grouping system and third-party headers at the top with angle brackets, relocate project headers appropriately, and eliminate redundant or unused includes. Additionally, streamline source files by repositioning self-include statements, adjust comment formatting, correct minor typos, and remove superfluous whitespace or blank lines for improved readability and consistency without altering functionality.
* refactor(yasmin_pcl): streamline pcl_conversions usage by removing compat header
  Eliminate the custom pcl_conversions_compat.hpp header and directly include pcl_conversions.h in source files, while updating CMakeLists.txt to conditionally use target-based or variable-based dependencies for better ROS version support. This simplifies the codebase without altering functionality.
* feat(yasmin_pcl): add compatibility header for pcl_conversions and update includes
* build(yasmin_pcl): conditionalize gtest dependencies on BUILD_TESTING
  Move ament_cmake_gtest find_package and test additions inside BUILD_TESTING check to allow building without tests.
* feat(yasmin_pcl): add PCL compatibility header for versions prior to 1.12
  Add pcl_compat.hpp to provide type aliases for pcl::Indices and related types in PCL versions older than 1.12. Update includes in cloud_types.hpp and filter_state_utils.hpp to use the new header. Refactor CMakeLists.txt to use direct linking for dependencies. Add new test files and fix use-after-free in factory cleanup test.
* Update build configurations and README to include yasmin_pcl package
* Yasmin pcl (`#96 <https://github.com/uleroboticsgroup/yasmin/issues/96>`_)
  * add yasmin_pcl
  * add more tests and add license
  * fix tests
  ---------
  Co-authored-by: Maik Knof <knofm@hs-weingarten.de>
* Contributors: Miguel Ángel González Santamarta
