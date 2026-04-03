from setuptools import find_packages, setup

package_name = "yasmin_cli"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "ros2cli"],
    zip_safe=True,
    maintainer="Maik Knof",
    maintainer_email="maik.knof@gmx.de",
    description="ROS 2 CLI extensions for YASMIN.",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "yasmin = yasmin_cli.command.yasmin:YasminCommand",
        ],
        "ros2cli.extension_point": [
            "yasmin = ros2cli.command.CommandExtension",
        ],
    },
)
