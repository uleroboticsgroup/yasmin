from setuptools import setup

package_name = "yasmin_plugins_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maik Knof",
    maintainer_email="maik.knof@gmx.de",
    description="Plugin discovery and caching for YASMIN states.",
    license="GPL-3.0",
    entry_points={
        "console_scripts": [
            "discover_plugins = yasmin_plugins_manager.discovery_node:main",
        ],
    },
)
