^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yasmin_pcl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
