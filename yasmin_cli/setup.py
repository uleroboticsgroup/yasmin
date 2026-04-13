from setuptools import find_packages, setup

package_name = "yasmin_cli"

setup(
    name=package_name,
    version="5.0.0",
    packages=find_packages(exclude=["test"]),
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
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "ros2cli.command": [
            "yasmin = yasmin_cli.command.yasmin:YasminCommand",
        ],
        "ros2cli.extension_point": [
            "yasmin = ros2cli.command.CommandExtension",
        ],
    },
)
