#!/bin/bash

pkg=yasmin_ros
ros_pkg_path=$(python3 -c "from ament_index_python.packages import get_package_prefix; print(get_package_prefix('$pkg'))")
py_pkg_path=$(python3 -c "import os, $pkg; print(os.path.dirname($pkg.__file__))")
python3 -m pytest --cov=$py_pkg_path $ros_pkg_path/pytests/$pkg/*.py
